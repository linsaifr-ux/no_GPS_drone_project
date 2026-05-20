#!/usr/bin/env python3
"""
Build AnyLoc geo-tagged image database from Taiwan NLSC satellite orthophoto.

Creates anyloc/database/ with:
  database.pt    — torch save: lats, lons, alts, vlads (N×D), codebook (k×768)
  db_images/     — satellite crop JPEGs

Run once before starting the localizer:
    conda run -n isaac_sim_test python anyloc/build_database.py

Options:
    --grid-step 50    grid spacing in metres (default 50)
    --agl 50          drone AGL in metres (sets footprint size, default 50)
    --rebuild         overwrite existing database
"""

import argparse, math, os, sys
import numpy as np
from PIL import Image
import torch
import faiss

HERE    = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(HERE, '..', 'simulator'))
DB_DIR  = os.path.join(HERE, 'database')
IMG_DIR = os.path.join(DB_DIR, 'db_images')

# Must match simulator/cesium_scene.py
CENTER_LAT = 23.450868
CENTER_LON = 120.286135
RADIUS_M   = 2000.0
R_EARTH    = 6_371_000.0
COS_LAT    = math.cos(math.radians(CENTER_LAT))
SAT_ZOOM   = 18

# Drone camera: 18 mm / 36×27 mm aperture → 90°×73.7° FOV
HFOV_DEG = 90.0
VFOV_DEG = 73.7

# AnyLoc VLAD settings
DINO_IMG_W = 448    # must be divisible by 14 (ViT-B/14 patch size)
DINO_IMG_H = 336
VLAD_K     = 64


# ── Geo helpers ────────────────────────────────────────────────────────────────

def to_latlon(x_enu, y_enu):
    lat = CENTER_LAT + (y_enu / R_EARTH) * (180.0 / math.pi)
    lon = CENTER_LON + (x_enu / (R_EARTH * COS_LAT)) * (180.0 / math.pi)
    return lat, lon


# ── Satellite tile bounds (replicates fetch_satellite() in cesium_scene.py) ───

def _deg2tile(lat, lon, z):
    n  = 1 << z
    x  = int((lon + 180.0) / 360.0 * n)
    lr = math.log(math.tan(math.radians(lat)) + 1.0 / math.cos(math.radians(lat)))
    y  = int((1.0 - lr / math.pi) / 2.0 * n)
    return x, y


def _tile2deg(tx, ty, z):
    n   = 1 << z
    lon = tx / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * ty / n))))
    return lat, lon


def compute_sat_bounds(margin_factor=1.5):
    d_lat = RADIUS_M * margin_factor / 111_320.0
    d_lon = RADIUS_M * margin_factor / (111_320.0 * COS_LAT)
    tx_min, ty_min = _deg2tile(CENTER_LAT + d_lat, CENTER_LON - d_lon, SAT_ZOOM)
    tx_max, ty_max = _deg2tile(CENTER_LAT - d_lat, CENTER_LON + d_lon, SAT_ZOOM)
    nw_lat, nw_lon = _tile2deg(tx_min,     ty_min,     SAT_ZOOM)
    se_lat, se_lon = _tile2deg(tx_max + 1, ty_max + 1, SAT_ZOOM)
    return dict(nw_lat=nw_lat, nw_lon=nw_lon, se_lat=se_lat, se_lon=se_lon)


# ── Satellite crop ─────────────────────────────────────────────────────────────

def get_satellite_crop(sat_img, bounds, lat, lon, agl_m, out_size=(640, 480)):
    """
    Return the satellite patch corresponding to a nadir drone view from (lat, lon, agl_m).
    Image coords: x=0 at west, y=0 at north (top-left = NW corner).
    """
    half_w_m = agl_m * math.tan(math.radians(HFOV_DEG / 2.0))
    half_h_m = agl_m * math.tan(math.radians(VFOV_DEG / 2.0))
    d_lat = half_h_m / 111_320.0
    d_lon = half_w_m / (111_320.0 * COS_LAT)

    img_w, img_h = sat_img.size
    lon_span = bounds['se_lon'] - bounds['nw_lon']
    lat_span = bounds['nw_lat'] - bounds['se_lat']   # positive (north − south)

    x1 = int((lon - d_lon - bounds['nw_lon']) / lon_span * img_w)
    x2 = int((lon + d_lon - bounds['nw_lon']) / lon_span * img_w)
    y1 = int((bounds['nw_lat'] - (lat + d_lat)) / lat_span * img_h)
    y2 = int((bounds['nw_lat'] - (lat - d_lat)) / lat_span * img_h)

    x1c, y1c = max(0, x1), max(0, y1)
    x2c, y2c = min(img_w, x2), min(img_h, y2)
    if x2c - x1c < 10 or y2c - y1c < 10:
        return None

    return sat_img.crop((x1c, y1c, x2c, y2c)).resize(out_size, Image.LANCZOS)


# ── PIL → torch tensor (avoids numpy dual-install issues) ─────────────────────

_MEAN = (0.485, 0.456, 0.406)
_STD  = (0.229, 0.224, 0.225)


def pil_to_tensor(pil_img):
    """PIL RGB → (C, H, W) normalised float32 tensor using PIL tobytes + frombuffer."""
    img = pil_img.resize((DINO_IMG_W, DINO_IMG_H), Image.LANCZOS).convert('RGB')
    t = torch.frombuffer(bytearray(img.tobytes()), dtype=torch.uint8) \
              .reshape(DINO_IMG_H, DINO_IMG_W, 3).float() / 255.0
    mean = torch.tensor(_MEAN).view(1, 1, 3)
    std  = torch.tensor(_STD).view(1, 1, 3)
    return ((t - mean) / std).permute(2, 0, 1)   # (H,W,C) → (C,H,W)


# ── numpy array → torch via buffer (avoids torch.from_numpy type check) ───────

def np_to_tensor(arr):
    """Convert a numpy float32 array to a torch tensor via raw bytes."""
    return torch.frombuffer(bytearray(arr.tobytes()), dtype=torch.float32) \
                .reshape(arr.shape)


# ── DINOv2 feature extraction ──────────────────────────────────────────────────

def load_dino(device):
    print(f"[DB] Loading DINOv2 ViT-B/14 on {device} …")
    model = torch.hub.load('facebookresearch/dinov2', 'dinov2_vitb14', pretrained=True)
    model.eval().to(device)
    return model


def extract_features(images, model, device, batch=8):
    """Return list of (N_patches, 768) float32 CPU tensors, one per image."""
    all_feats = []
    for i in range(0, len(images), batch):
        chunk = images[i:i + batch]
        x = torch.stack([pil_to_tensor(img) for img in chunk]).to(device)
        with torch.no_grad():
            out = model.forward_features(x)
        # out['x_norm_patchtokens']: (B, N_patches, 768)
        patches = out['x_norm_patchtokens'].cpu()   # keep as torch tensor
        for j in range(len(chunk)):
            all_feats.append(patches[j].float())
        if (i + batch) % 80 < batch:
            print(f"  features: {min(i + batch, len(images))}/{len(images)}")
    return all_feats


# ── VLAD codebook + descriptors (pure torch) ──────────────────────────────────

def build_codebook(all_feats_list, k=VLAD_K):
    """k-means via faiss on all patch features from the database images."""
    all_flat = torch.cat(all_feats_list, dim=0)   # (total_patches, 768)
    d = all_flat.shape[1]
    print(f"[DB] faiss k-means: {len(all_flat)} patches → k={k} clusters …")
    km = faiss.Kmeans(d, k, niter=50, verbose=False, gpu=False)
    km.train(all_flat.numpy())    # faiss needs numpy; .numpy() doesn't call numpy methods
    # centroids: numpy (k, d) → torch via frombuffer
    return np_to_tensor(km.centroids.astype('f4'))   # (k, d) float32 torch tensor


def compute_vlad(feats, codebook):
    """
    Intra-normalised VLAD descriptor, fully in torch.
    feats:    (N, d) float32 CPU tensor
    codebook: (k, d) float32 CPU tensor
    Returns:  (k*d,) float32 CPU tensor, L2-normalised
    """
    k, d = codebook.shape
    v = torch.zeros(k, d, dtype=torch.float32)

    ff = (feats ** 2).sum(1, keepdim=True)                        # (N, 1)
    cc = (codebook ** 2).sum(1, keepdim=True)                     # (k, 1)
    dists   = ff + cc.T - 2.0 * (feats @ codebook.T)             # (N, k)
    assigns = dists.argmin(1)                                      # (N,)

    for c in range(k):
        mask = assigns == c
        if mask.any():
            v[c] = (feats[mask] - codebook[c]).sum(0)

    norms = v.norm(dim=1, keepdim=True)
    v = v / (norms + 1e-8)
    v = v.flatten()
    v = v / (v.norm() + 1e-8)
    return v.float()


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--grid-step', type=float, default=50.0,
                    help='Grid spacing in metres (default 50)')
    ap.add_argument('--agl', type=float, default=50.0,
                    help='Drone AGL in metres for footprint calculation (default 50)')
    ap.add_argument('--rebuild', action='store_true',
                    help='Rebuild even if database.pt already exists')
    args = ap.parse_args()

    db_file = os.path.join(DB_DIR, 'database.pt')
    if os.path.exists(db_file) and not args.rebuild:
        print(f"[DB] {db_file} already exists.  Use --rebuild to regenerate.")
        return

    # Load satellite image
    sat_path = os.path.join(SIM_DIR, 'satellite_ground.jpg')
    if not os.path.exists(sat_path):
        sys.exit(f"[DB] Satellite image not found: {sat_path}\n"
                 "     Run simulator/cesium_scene.py once to download it.")
    sat_img = Image.open(sat_path).convert('RGB')
    bounds  = compute_sat_bounds()
    print(f"[DB] Satellite: {sat_img.size[0]}×{sat_img.size[1]}")
    print(f"[DB] Bounds:  {bounds['nw_lat']:.5f}°N – {bounds['se_lat']:.5f}°N  "
          f"{bounds['nw_lon']:.5f}°E – {bounds['se_lon']:.5f}°E")

    # Grid positions (filter to a circle that fits inside the satellite image)
    step  = args.grid_step
    agl   = args.agl
    limit = RADIUS_M * 0.75
    coords = [(x, y)
              for x in np.arange(-limit, limit + 1, step)
              for y in np.arange(-limit, limit + 1, step)
              if math.hypot(x, y) <= limit]
    print(f"[DB] Grid step={step} m  AGL={agl} m  →  {len(coords)} positions")

    # Crop and save database images
    os.makedirs(IMG_DIR, exist_ok=True)
    db_lats, db_lons, db_alts = [], [], []
    db_images = []
    skipped   = 0
    for x_enu, y_enu in coords:
        lat, lon = to_latlon(x_enu, y_enu)
        crop = get_satellite_crop(sat_img, bounds, lat, lon, agl)
        if crop is None:
            skipped += 1
            continue
        idx = len(db_images)
        crop.save(os.path.join(IMG_DIR, f'{idx:06d}.jpg'), 'JPEG', quality=90)
        db_lats.append(lat)
        db_lons.append(lon)
        db_alts.append(agl)
        db_images.append(crop)
    print(f"[DB] Cropped {len(db_images)} patches  ({skipped} outside bounds)")

    if len(db_images) < VLAD_K:
        sys.exit(f"[DB] Too few images ({len(db_images)}) to build a k={VLAD_K} codebook. "
                 "Reduce --grid-step or --agl.")

    # DINOv2 features (returned as torch CPU tensors)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model  = load_dino(device)
    print(f"[DB] Extracting patch features …")
    all_feats = extract_features(db_images, model, device)

    # VLAD codebook
    codebook = build_codebook(all_feats, k=VLAD_K)
    print(f"[DB] Codebook: {tuple(codebook.shape)}")

    # VLAD descriptors for all database images
    print(f"[DB] Computing VLAD descriptors …")
    vlads = torch.stack([compute_vlad(f, codebook) for f in all_feats])
    print(f"[DB] VLAD matrix: {tuple(vlads.shape)}  (dim={vlads.shape[1]})")

    # Save using torch.save (avoids numpy savez issues)
    os.makedirs(DB_DIR, exist_ok=True)
    torch.save({
        'lats':     torch.tensor(db_lats, dtype=torch.float32),
        'lons':     torch.tensor(db_lons, dtype=torch.float32),
        'alts':     torch.tensor(db_alts, dtype=torch.float32),
        'vlads':    vlads,
        'codebook': codebook,
    }, db_file)
    print(f"[DB] Saved → {db_file}")
    print(f"[DB] Done: {len(db_lats)} entries, VLAD dim={vlads.shape[1]}")


if __name__ == '__main__':
    main()
