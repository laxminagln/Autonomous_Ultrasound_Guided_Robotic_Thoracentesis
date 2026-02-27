import cv2
import numpy as np

mask_path = r"C:\Users\laxmi\Downloads\MSc Dissertation\Dataset\PleuralEffusionDataset\masks_multiclass\img_192.jpg"
m = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
print("unique values:", np.unique(m))
print("min/max:", m.min(), m.max())