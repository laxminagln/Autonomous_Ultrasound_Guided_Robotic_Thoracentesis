import os
import random
from pathlib import Path

import cv2


# ======================================================
# PATHS (Windows) - EDIT ONLY IF NEEDED
# ======================================================
INPUT_DIR = r"C:\Users\laxmi\Downloads\MSc Dissertation\Dataset\PleuralEffusionDataset\images"
OUTPUT_DIR = r"C:\Users\laxmi\Downloads\MSc Dissertation\Dataset\PleuralEffusionDataset\images_cropped"

# ======================================================
# CROP SETTINGS (EDIT THESE ONCE YOU KNOW THE ROI)
# ======================================================
# These are example values. If your crop exceeds any image size, the script
# will either clamp or skip depending on CLAMP_CROP_TO_IMAGE.
TOP = 124
BOTTOM = 626
LEFT = 271
RIGHT = 753

# If True: crop box is clamped to each image boundary (never out-of-range)
# If False: images that don't fit the crop box are skipped
CLAMP_CROP_TO_IMAGE = True

# ======================================================
# RESIZE SETTINGS
# ======================================================
RESIZE = True
TARGET_SIZE = (256, 256)  # (width, height)

# ======================================================
# PREVIEW MODE
# ======================================================
# If True: shows preview for a few random images before batch processing.
# Press any key to continue to next preview; press ESC to stop preview early.
PREVIEW_BEFORE_RUN = True
PREVIEW_SAMPLES = 5

# ======================================================
# IMAGE TYPES
# ======================================================
EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def list_images(folder: str):
    p = Path(folder)
    files = []
    for f in p.iterdir():
        if f.is_file() and f.suffix.lower() in EXTS:
            files.append(str(f))
    return sorted(files)


def clamp_box(top, bottom, left, right, h, w):
    """Clamp crop coordinates to [0,h] and [0,w]."""
    top_c = max(0, min(top, h))
    bottom_c = max(0, min(bottom, h))
    left_c = max(0, min(left, w))
    right_c = max(0, min(right, w))
    return top_c, bottom_c, left_c, right_c


def is_valid_box(top, bottom, left, right):
    return (bottom > top) and (right > left)


def preview_crop(image_paths):
    if not image_paths:
        print("No images found to preview.")
        return

    samples = image_paths[:]
    random.shuffle(samples)
    samples = samples[: min(PREVIEW_SAMPLES, len(samples))]

    for path in samples:
        img = cv2.imread(path)
        if img is None:
            print(f"[PREVIEW] Unreadable: {path}")
            continue

        h, w = img.shape[:2]

        if CLAMP_CROP_TO_IMAGE:
            t, b, l, r = clamp_box(TOP, BOTTOM, LEFT, RIGHT, h, w)
        else:
            t, b, l, r = TOP, BOTTOM, LEFT, RIGHT

        # Draw rectangle on a copy
        vis = img.copy()
        cv2.rectangle(vis, (l, t), (r, b), (0, 255, 0), 2)

        # Make a cropped view too
        crop = vis[t:b, l:r] if is_valid_box(t, b, l, r) else None

        cv2.imshow("Preview - Full image with crop box", vis)
        if crop is not None and crop.size > 0:
            cv2.imshow("Preview - Cropped region", crop)
        else:
            blank = (vis * 0)  # black
            cv2.putText(blank, "EMPTY CROP", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("Preview - Cropped region", blank)

        key = cv2.waitKey(0) & 0xFF
        cv2.destroyWindow("Preview - Full image with crop box")
        cv2.destroyWindow("Preview - Cropped region")

        # ESC to exit preview early
        if key == 27:
            break


def main():
    in_dir = Path(INPUT_DIR)
    out_dir = Path(OUTPUT_DIR)
    out_dir.mkdir(parents=True, exist_ok=True)

    image_paths = list_images(str(in_dir))
    if not image_paths:
        print(f"❌ No images found in: {in_dir}")
        return

    print(f"✅ Found {len(image_paths)} images in: {in_dir}")

    if PREVIEW_BEFORE_RUN:
        print("\n--- PREVIEW MODE ---")
        print("A few random images will show with the crop box.")
        print("Press any key to go to next preview. Press ESC to exit preview.\n")
        preview_crop(image_paths)

    processed = 0
    skipped_unreadable = 0
    skipped_invalid_crop = 0
    skipped_too_small = 0

    print("\n--- BATCH CROPPING START ---\n")

    for img_path in image_paths:
        img = cv2.imread(img_path)
        filename = Path(img_path).name

        if img is None:
            print(f"[SKIP] Unreadable: {filename}")
            skipped_unreadable += 1
            continue

        h, w = img.shape[:2]

        if CLAMP_CROP_TO_IMAGE:
            t, b, l, r = clamp_box(TOP, BOTTOM, LEFT, RIGHT, h, w)
            if not is_valid_box(t, b, l, r):
                print(f"[SKIP] Invalid crop after clamping for {filename} | img={h}x{w} box=({t},{b},{l},{r})")
                skipped_invalid_crop += 1
                continue
        else:
            # If not clamping, enforce box must fit
            if BOTTOM > h or RIGHT > w:
                print(f"[SKIP] Too small for crop: {filename} | img={h}x{w} needs bottom<= {h}, right<= {w}")
                skipped_too_small += 1
                continue
            t, b, l, r = TOP, BOTTOM, LEFT, RIGHT
            if not is_valid_box(t, b, l, r):
                print(f"[SKIP] Invalid crop box: {filename} | box=({t},{b},{l},{r})")
                skipped_invalid_crop += 1
                continue

        cropped = img[t:b, l:r]
        if cropped is None or cropped.size == 0:
            print(f"[SKIP] Empty crop: {filename} | img={h}x{w} box=({t},{b},{l},{r})")
            skipped_invalid_crop += 1
            continue

        if RESIZE:
            try:
                cropped = cv2.resize(cropped, TARGET_SIZE, interpolation=cv2.INTER_AREA)
            except cv2.error as e:
                print(f"[SKIP] Resize failed: {filename} | {e}")
                skipped_invalid_crop += 1
                continue

        out_path = out_dir / filename
        ok = cv2.imwrite(str(out_path), cropped)
        if not ok:
            print(f"[SKIP] Failed to write: {filename}")
            skipped_invalid_crop += 1
            continue

        processed += 1
        if processed % 50 == 0:
            print(f"... processed {processed}/{len(image_paths)}")

    print("\n--- DONE ---")
    print(f"✅ Processed: {processed}")
    print(f"⏭️ Skipped unreadable: {skipped_unreadable}")
    print(f"⏭️ Skipped too small (no clamp): {skipped_too_small}")
    print(f"⏭️ Skipped invalid/empty crop: {skipped_invalid_crop}")
    print(f"\n📁 Output folder: {out_dir}")


if __name__ == "__main__":
    main()