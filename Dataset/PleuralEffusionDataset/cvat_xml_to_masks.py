import os
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np


# =========================
# EDIT THESE PATHS
# =========================
IMAGES_DIR = r"C:\Users\laxmi\Downloads\MSc Dissertation\Dataset\PleuralEffusionDataset\images"
CVAT_XML   = r"C:\Users\laxmi\Downloads\MSc Dissertation\Dataset\PleuralEffusionDataset\annotations.xml"

OUT_RAW = r"C:\Users\laxmi\Downloads\MSc Dissertation\Dataset\PleuralEffusionDataset\masks_multiclass_raw"
OUT_VIS = r"C:\Users\laxmi\Downloads\MSc Dissertation\Dataset\PleuralEffusionDataset\masks_multiclass_vis"

# Optional binary masks (useful for checking)
OUT_EFF  = r"C:\Users\laxmi\Downloads\MSc Dissertation\Dataset\PleuralEffusionDataset\masks_binary_effusion"
OUT_BONE = r"C:\Users\laxmi\Downloads\MSc Dissertation\Dataset\PleuralEffusionDataset\masks_binary_bone"


# =========================
# STRICT class IDs (0/1/2 only)
# =========================
BACKGROUND = 0
EFFUSION  = 1
BONE      = 2

LABEL_EFFUSION = "Effusion"
LABEL_BONE     = "Bone"


def parse_points(points_str: str) -> np.ndarray:
    """CVAT polygon points format: 'x1,y1;x2,y2;...' -> Nx2 float array."""
    pts = []
    for p in points_str.strip().split(";"):
        if not p:
            continue
        x, y = p.split(",")
        pts.append([float(x), float(y)])
    return np.array(pts, dtype=np.float32)


def clamp_int_points(pts_xy: np.ndarray, w: int, h: int) -> np.ndarray:
    """Round to int and clamp within image bounds."""
    pts = np.round(pts_xy).astype(np.int32)
    pts[:, 0] = np.clip(pts[:, 0], 0, w - 1)
    pts[:, 1] = np.clip(pts[:, 1], 0, h - 1)
    return pts


def make_vis(mask_raw: np.ndarray) -> np.ndarray:
    """Map 0/1/2 to 0/127/255 so it looks visible."""
    vis = np.zeros_like(mask_raw, dtype=np.uint8)
    vis[mask_raw == EFFUSION] = 127
    vis[mask_raw == BONE] = 255
    return vis


def main():
    # Create output folders
    Path(OUT_RAW).mkdir(parents=True, exist_ok=True)
    Path(OUT_VIS).mkdir(parents=True, exist_ok=True)
    Path(OUT_EFF).mkdir(parents=True, exist_ok=True)
    Path(OUT_BONE).mkdir(parents=True, exist_ok=True)

    if not os.path.exists(CVAT_XML):
        raise FileNotFoundError(f"CVAT XML not found: {CVAT_XML}")

    # Parse XML
    tree = ET.parse(CVAT_XML)
    root = tree.getroot()

    # Index images on disk
    disk_images = {fn: os.path.join(IMAGES_DIR, fn) for fn in os.listdir(IMAGES_DIR)}

    unknown_labels = set()
    saved = 0
    skipped = 0

    # CVAT format: <annotations> ... <image ...> <polygon .../> </image>
    for img_node in root.findall("./image"):
        img_name = img_node.get("name")
        if not img_name:
            continue

        img_basename = os.path.basename(img_name)

        if img_basename not in disk_images:
            print(f"[SKIP] Not on disk: {img_basename}")
            skipped += 1
            continue

        img_path = disk_images[img_basename]
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"[SKIP] Unreadable: {img_basename}")
            skipped += 1
            continue

        h, w = img.shape[:2]

        # Raw multiclass mask (0/1/2) + binary masks
        mask_raw = np.zeros((h, w), dtype=np.uint8)
        mask_eff = np.zeros((h, w), dtype=np.uint8)
        mask_bone = np.zeros((h, w), dtype=np.uint8)

        polygons = list(img_node.findall("polygon"))

        # ---- Pass 1: Bone (fill first)
        for poly in polygons:
            label = (poly.get("label") or "").strip()
            pts_str = poly.get("points") or ""
            if not pts_str:
                continue

            pts = clamp_int_points(parse_points(pts_str), w, h).reshape((-1, 1, 2))

            if label == LABEL_BONE:
                cv2.fillPoly(mask_raw, [pts], BONE)
                cv2.fillPoly(mask_bone, [pts], 255)
            elif label != LABEL_EFFUSION and label:
                unknown_labels.add(label)

        # ---- Pass 2: Effusion (overwrite bone where effusion exists)
        for poly in polygons:
            label = (poly.get("label") or "").strip()
            pts_str = poly.get("points") or ""
            if not pts_str:
                continue

            pts = clamp_int_points(parse_points(pts_str), w, h).reshape((-1, 1, 2))

            if label == LABEL_EFFUSION:
                cv2.fillPoly(mask_raw, [pts], EFFUSION)
                cv2.fillPoly(mask_eff, [pts], 255)

        # Visible version for you to view
        mask_vis = make_vis(mask_raw)

        # ✅ IMPORTANT: save masks as PNG (lossless) even if images are JPG
        stem = Path(img_basename).stem
        out_name = f"{stem}.png"

        cv2.imwrite(os.path.join(OUT_RAW, out_name), mask_raw)    # 0/1/2
        cv2.imwrite(os.path.join(OUT_VIS, out_name), mask_vis)    # 0/127/255
        cv2.imwrite(os.path.join(OUT_EFF, out_name), mask_eff)    # 0/255
        cv2.imwrite(os.path.join(OUT_BONE, out_name), mask_bone)  # 0/255

        saved += 1
        if saved % 50 == 0:
            print(f"... saved {saved}")

    print("\n✅ Done")
    print(f"Saved: {saved} | Skipped: {skipped}")

    if unknown_labels:
        print("⚠️ Unknown labels found (should be none):", sorted(unknown_labels))

    # Sanity check: read one RAW mask and show unique values
    raw_files = sorted([f for f in os.listdir(OUT_RAW) if f.lower().endswith(".png")])
    if raw_files:
        test_path = os.path.join(OUT_RAW, raw_files[0])
        m = cv2.imread(test_path, cv2.IMREAD_GRAYSCALE)
        print(f"\nSanity check: {raw_files[0]}")
        print("Unique values:", np.unique(m), "min/max:", int(m.min()), int(m.max()))
        if int(m.max()) > 2:
            print("❌ ERROR: found values > 2. This should not happen with PNG masks.")
        else:
            print("✅ OK: mask values are within 0..2 (perfect for UNet)")


if __name__ == "__main__":
    main()