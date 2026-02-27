import os
import random
import shutil
from pathlib import Path
import csv

# =========================
# EDIT THESE PATHS
# =========================
BASE = r"C:\Users\laxmi\Downloads\MSc Dissertation\Dataset\PleuralEffusionDataset"

IMAGES_DIR = os.path.join(BASE, "images")          # your cropped JPG images
MASKS_DIR  = os.path.join(BASE, "masks_multiclass_raw")    # your RAW PNG masks (0/1/2)

OUT_BASE   = os.path.join(BASE, "dataset_unet")

# Split ratios
TRAIN_RATIO = 0.80
VAL_RATIO   = 0.10
TEST_RATIO  = 0.10

SEED = 42

# Acceptable image extensions
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}

def ensure_dirs():
    for split in ["train", "val", "test"]:
        Path(os.path.join(OUT_BASE, "images", split)).mkdir(parents=True, exist_ok=True)
        Path(os.path.join(OUT_BASE, "masks", split)).mkdir(parents=True, exist_ok=True)
    Path(os.path.join(OUT_BASE, "meta")).mkdir(parents=True, exist_ok=True)

def find_pairs():
    """
    Match image file to mask file by stem.
    Example: img_001.jpg  -> masks/img_001.png
    """
    images = []
    for fn in os.listdir(IMAGES_DIR):
        p = Path(fn)
        if p.suffix.lower() in IMG_EXTS:
            images.append(fn)

    pairs = []
    missing_masks = []
    for img_fn in sorted(images):
        stem = Path(img_fn).stem
        mask_fn = stem + ".png"  # masks are PNG
        mask_path = os.path.join(MASKS_DIR, mask_fn)
        if os.path.exists(mask_path):
            pairs.append((img_fn, mask_fn))
        else:
            missing_masks.append(img_fn)

    return pairs, missing_masks

def split_list(items):
    random.seed(SEED)
    items = items[:]
    random.shuffle(items)

    n = len(items)
    n_train = int(n * TRAIN_RATIO)
    n_val = int(n * VAL_RATIO)
    # remainder goes to test
    train = items[:n_train]
    val = items[n_train:n_train+n_val]
    test = items[n_train+n_val:]

    return train, val, test

def copy_pairs(pairs, split_name):
    out_img_dir = os.path.join(OUT_BASE, "images", split_name)
    out_msk_dir = os.path.join(OUT_BASE, "masks", split_name)

    for img_fn, mask_fn in pairs:
        src_img = os.path.join(IMAGES_DIR, img_fn)
        src_msk = os.path.join(MASKS_DIR, mask_fn)

        dst_img = os.path.join(out_img_dir, img_fn)
        dst_msk = os.path.join(out_msk_dir, mask_fn)

        shutil.copy2(src_img, dst_img)
        shutil.copy2(src_msk, dst_msk)

def write_meta_csv(train, val, test):
    meta_path = os.path.join(OUT_BASE, "meta", "split.csv")
    with open(meta_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["split", "image", "mask"])
        for split_name, lst in [("train", train), ("val", val), ("test", test)]:
            for img_fn, mask_fn in lst:
                w.writerow([split_name, img_fn, mask_fn])

def write_classes():
    classes_path = os.path.join(OUT_BASE, "meta", "classes.txt")
    with open(classes_path, "w", encoding="utf-8") as f:
        f.write("0 background\n")
        f.write("1 effusion\n")
        f.write("2 bone\n")

def main():
    if not os.path.isdir(IMAGES_DIR):
        raise FileNotFoundError(f"Images folder not found: {IMAGES_DIR}")
    if not os.path.isdir(MASKS_DIR):
        raise FileNotFoundError(f"Masks folder not found: {MASKS_DIR}")

    ensure_dirs()

    pairs, missing = find_pairs()
    print(f"Found pairs: {len(pairs)}")
    if missing:
        print(f"WARNING: {len(missing)} images have no mask. Examples:")
        for ex in missing[:10]:
            print(" -", ex)
        print("Fix by ensuring masks exist for every image stem.")

    train, val, test = split_list(pairs)
    print(f"Split: train={len(train)} val={len(val)} test={len(test)}")

    # Copy into output dataset
    copy_pairs(train, "train")
    copy_pairs(val, "val")
    copy_pairs(test, "test")

    # Write metadata
    write_meta_csv(train, val, test)
    write_classes()

    print("\n✅ Dataset created at:")
    print(OUT_BASE)
    print("\nNext: train UNet using:")
    print(os.path.join(OUT_BASE, "images", "train"))
    print(os.path.join(OUT_BASE, "masks", "train"))

if __name__ == "__main__":
    main()