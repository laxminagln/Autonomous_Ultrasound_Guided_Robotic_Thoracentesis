import os
from pathlib import Path
import random
import numpy as np
import cv2
from tqdm import tqdm
import matplotlib.pyplot as plt

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader


# =========================
# CONFIG
# =========================
DATASET_ROOT = r"C:\Users\laxmi\Downloads\MSc Dissertation\Dataset\PleuralEffusionDataset\dataset_unet"

NUM_CLASSES = 3  # 0 background, 1 effusion, 2 bone
CLASS_BACKGROUND = 0
CLASS_EFFUSION  = 1
CLASS_BONE      = 2

IMG_SIZE = 256
BATCH_SIZE = 4
EPOCHS = 40
LR = 1e-3
SEED = 42

# Loss weighting (helps class imbalance).
# If bone is rare, give it a higher weight.
USE_CLASS_WEIGHTS = True
CLASS_WEIGHTS = torch.tensor([1.0, 2.0, 4.0], dtype=torch.float32)  # bg, eff, bone (tweak if needed)

SAVE_DIR = os.path.join(DATASET_ROOT, "runs_unet")
Path(SAVE_DIR).mkdir(parents=True, exist_ok=True)

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


def set_seed(seed=42):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


# =========================
# Dataset
# =========================
class USSegDataset(Dataset):
    def __init__(self, img_dir, mask_dir, augment=False):
        self.img_dir = Path(img_dir)
        self.mask_dir = Path(mask_dir)
        self.augment = augment

        self.images = sorted([p for p in self.img_dir.iterdir()
                              if p.suffix.lower() in [".jpg", ".jpeg", ".png"]])
        if len(self.images) == 0:
            raise RuntimeError(f"No images found in {img_dir}")

    def __len__(self):
        return len(self.images)

    def _augment(self, img, mask):
        if random.random() < 0.5:
            img = np.fliplr(img).copy()
            mask = np.fliplr(mask).copy()

        if random.random() < 0.3:
            img = np.flipud(img).copy()
            mask = np.flipud(mask).copy()

        if random.random() < 0.3:
            angle = random.uniform(-10, 10)
            h, w = img.shape[:2]
            M = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1.0)
            img = cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
            mask = cv2.warpAffine(mask, M, (w, h), flags=cv2.INTER_NEAREST,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        return img, mask

    def __getitem__(self, idx):
        img_path = self.images[idx]
        stem = img_path.stem
        mask_path = self.mask_dir / f"{stem}.png"

        img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f"Failed to read image: {img_path}")

        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise RuntimeError(f"Failed to read mask: {mask_path}")

        if img.shape != (IMG_SIZE, IMG_SIZE):
            img = cv2.resize(img, (IMG_SIZE, IMG_SIZE), interpolation=cv2.INTER_LINEAR)
        if mask.shape != (IMG_SIZE, IMG_SIZE):
            mask = cv2.resize(mask, (IMG_SIZE, IMG_SIZE), interpolation=cv2.INTER_NEAREST)

        # Ensure mask is strictly 0..NUM_CLASSES-1
        mask = np.clip(mask, 0, NUM_CLASSES - 1).astype(np.uint8)

        if self.augment:
            img, mask = self._augment(img, mask)

        img = img.astype(np.float32) / 255.0  # [0,1]

        img_t = torch.from_numpy(img).unsqueeze(0)          # [1,H,W]
        mask_t = torch.from_numpy(mask.astype(np.int64))    # [H,W]

        return img_t, mask_t, stem


# =========================
# UNet model
# =========================
class DoubleConv(nn.Module):
    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(in_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.net(x)


class UNet(nn.Module):
    def __init__(self, in_channels=1, num_classes=3, base=32):
        super().__init__()
        self.down1 = DoubleConv(in_channels, base)
        self.pool1 = nn.MaxPool2d(2)

        self.down2 = DoubleConv(base, base * 2)
        self.pool2 = nn.MaxPool2d(2)

        self.down3 = DoubleConv(base * 2, base * 4)
        self.pool3 = nn.MaxPool2d(2)

        self.down4 = DoubleConv(base * 4, base * 8)
        self.pool4 = nn.MaxPool2d(2)

        self.bottleneck = DoubleConv(base * 8, base * 16)

        self.up4 = nn.ConvTranspose2d(base * 16, base * 8, 2, stride=2)
        self.conv4 = DoubleConv(base * 16, base * 8)

        self.up3 = nn.ConvTranspose2d(base * 8, base * 4, 2, stride=2)
        self.conv3 = DoubleConv(base * 8, base * 4)

        self.up2 = nn.ConvTranspose2d(base * 4, base * 2, 2, stride=2)
        self.conv2 = DoubleConv(base * 4, base * 2)

        self.up1 = nn.ConvTranspose2d(base * 2, base, 2, stride=2)
        self.conv1 = DoubleConv(base * 2, base)

        self.out = nn.Conv2d(base, num_classes, 1)

    def forward(self, x):
        d1 = self.down1(x); p1 = self.pool1(d1)
        d2 = self.down2(p1); p2 = self.pool2(d2)
        d3 = self.down3(p2); p3 = self.pool3(d3)
        d4 = self.down4(p3); p4 = self.pool4(d4)

        bn = self.bottleneck(p4)

        u4 = self.up4(bn)
        x4 = self.conv4(torch.cat([u4, d4], dim=1))

        u3 = self.up3(x4)
        x3 = self.conv3(torch.cat([u3, d3], dim=1))

        u2 = self.up2(x3)
        x2 = self.conv2(torch.cat([u2, d2], dim=1))

        u1 = self.up1(x2)
        x1 = self.conv1(torch.cat([u1, d1], dim=1))

        return self.out(x1)


# =========================
# Dice (fixed for missing classes)
# =========================
def dice_per_class(pred, target, num_classes=3, eps=1e-6):
    """
    pred:   [B,H,W] int
    target: [B,H,W] int
    If a class is absent in BOTH pred and target for a sample, dice should be 1.0 (perfect).
    """
    dices = []
    for c in range(num_classes):
        p = (pred == c).float()
        t = (target == c).float()
        inter = (p * t).sum(dim=(1, 2))
        denom = p.sum(dim=(1, 2)) + t.sum(dim=(1, 2))

        # If denom==0 -> class absent in both -> dice=1
        dice = torch.where(denom > 0,
                           (2 * inter + eps) / (denom + eps),
                           torch.ones_like(denom))
        dices.append(dice.mean().item())
    return dices


# =========================
# Train / Eval
# =========================
def run_epoch(model, loader, criterion, optimizer=None):
    is_train = optimizer is not None
    model.train() if is_train else model.eval()

    total_loss = 0.0
    all_dice = np.zeros(NUM_CLASSES, dtype=np.float64)
    steps = 0

    for imgs, masks, _ in tqdm(loader, leave=False):
        imgs = imgs.to(DEVICE)
        masks = masks.to(DEVICE)

        if is_train:
            optimizer.zero_grad(set_to_none=True)

        with torch.set_grad_enabled(is_train):
            logits = model(imgs)  # [B,C,H,W]
            loss = criterion(logits, masks)

            if is_train:
                loss.backward()
                optimizer.step()

        total_loss += loss.item()

        pred = torch.argmax(logits, dim=1)
        d = dice_per_class(pred, masks, NUM_CLASSES)
        all_dice += np.array(d)
        steps += 1

    return total_loss / max(1, steps), (all_dice / max(1, steps))


def save_preview(model, loader, out_path, max_show=6):
    model.eval()
    imgs, masks, stems = next(iter(loader))
    imgs = imgs.to(DEVICE)
    with torch.no_grad():
        logits = model(imgs)
        pred = torch.argmax(logits, dim=1).cpu().numpy()

    imgs_np = (imgs.cpu().numpy()[:, 0] * 255).astype(np.uint8)
    masks_np = masks.numpy().astype(np.uint8)

    n = min(len(imgs_np), max_show)
    fig, axes = plt.subplots(n, 3, figsize=(9, 3 * n))
    if n == 1:
        axes = np.expand_dims(axes, 0)

    for i in range(n):
        axes[i, 0].imshow(imgs_np[i], cmap="gray")
        axes[i, 0].set_title(f"Image: {stems[i]}")
        axes[i, 0].axis("off")

        axes[i, 1].imshow(masks_np[i], vmin=0, vmax=NUM_CLASSES - 1)
        axes[i, 1].set_title("GT mask (0/1/2)")
        axes[i, 1].axis("off")

        axes[i, 2].imshow(pred[i], vmin=0, vmax=NUM_CLASSES - 1)
        axes[i, 2].set_title("Pred mask")
        axes[i, 2].axis("off")

    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    set_seed(SEED)
    print("Device:", DEVICE)

    train_ds = USSegDataset(
        img_dir=os.path.join(DATASET_ROOT, "images", "train"),
        mask_dir=os.path.join(DATASET_ROOT, "masks", "train"),
        augment=True,
    )
    val_ds = USSegDataset(
        img_dir=os.path.join(DATASET_ROOT, "images", "val"),
        mask_dir=os.path.join(DATASET_ROOT, "masks", "val"),
        augment=False,
    )
    test_ds = USSegDataset(
        img_dir=os.path.join(DATASET_ROOT, "images", "test"),
        mask_dir=os.path.join(DATASET_ROOT, "masks", "test"),
        augment=False,
    )

    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True, num_workers=0)
    val_loader   = DataLoader(val_ds, batch_size=BATCH_SIZE, shuffle=False, num_workers=0)
    test_loader  = DataLoader(test_ds, batch_size=BATCH_SIZE, shuffle=False, num_workers=0)

    model = UNet(in_channels=1, num_classes=NUM_CLASSES, base=32).to(DEVICE)
    optimizer = optim.Adam(model.parameters(), lr=LR)

    if USE_CLASS_WEIGHTS:
        w = CLASS_WEIGHTS.to(DEVICE)
        criterion = nn.CrossEntropyLoss(weight=w)
        print("Using class weights:", CLASS_WEIGHTS.tolist())
    else:
        criterion = nn.CrossEntropyLoss()

    best_val = -1.0
    best_path = os.path.join(SAVE_DIR, "best_unet.pt")
    history = []

    for epoch in range(1, EPOCHS + 1):
        print(f"\nEpoch {epoch}/{EPOCHS}")

        tr_loss, tr_dice = run_epoch(model, train_loader, criterion, optimizer)
        va_loss, va_dice = run_epoch(model, val_loader, criterion, optimizer=None)

        # Score = mean Dice of effusion + bone
        score = (va_dice[CLASS_EFFUSION] + va_dice[CLASS_BONE]) / 2.0

        print(f"Train loss: {tr_loss:.4f} | Dice(bg/eff/bone): {tr_dice}")
        print(f"Val   loss: {va_loss:.4f} | Dice(bg/eff/bone): {va_dice} | score={score:.4f}")

        history.append((epoch, tr_loss, va_loss, va_dice[0], va_dice[1], va_dice[2], score))

        # Preview each epoch
        preview_path = os.path.join(SAVE_DIR, f"preview_epoch_{epoch:02d}.png")
        save_preview(model, val_loader, preview_path)

        if score > best_val:
            best_val = score
            torch.save(model.state_dict(), best_path)
            print(f"✅ Saved best model: {best_path} (score={best_val:.4f})")

    # Final test
    print("\nLoading best model for test...")
    model.load_state_dict(torch.load(best_path, map_location=DEVICE))
    te_loss, te_dice = run_epoch(model, test_loader, criterion, optimizer=None)
    te_score = (te_dice[CLASS_EFFUSION] + te_dice[CLASS_BONE]) / 2.0
    print(f"TEST loss: {te_loss:.4f} | Dice(bg/eff/bone): {te_dice} | score={te_score:.4f}")

    # Save history CSV
    hist_path = os.path.join(SAVE_DIR, "history.csv")
    with open(hist_path, "w", encoding="utf-8") as f:
        f.write("epoch,train_loss,val_loss,val_dice_bg,val_dice_effusion,val_dice_bone,val_score\n")
        for row in history:
            f.write(",".join(map(str, row)) + "\n")

    print("\n✅ Done")
    print("Best model:", best_path)
    print("History:", hist_path)
    print("Previews saved in:", SAVE_DIR)


if __name__ == "__main__":
    main()