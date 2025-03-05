# utils.py

# Utility functions for loading and preprocessing images
from ranger_object_recognition.config import TARGET_SIZE, MIN_BOX_SIZE, TOP_K
import cv2
import numpy as np


def load_image_uint8(image_path, scale=1.0):
    """Loads an image from the given path, converts it to RGB, downsizes it if needed, and returns it in uint8 format."""
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Could not load image at {image_path}")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    if scale != 1.0:
        h, w = img.shape[:2]
        img = cv2.resize(img, (int(w * scale), int(h * scale)))
    return img

def preprocess_image_for_tflite(img, target_size):
    """Resizes an image to target_size and converts it to uint8 with a batch dimension."""
    img_resized = cv2.resize(img, target_size)
    img_uint8 = img_resized.astype(np.uint8)
    return np.expand_dims(img_uint8, axis=0)

def extract_region(image, box):
    """Extracts a region from the image based on the given box coordinates."""
    if not is_valid_box(box):
        return None
    x, y, w, h = box
    region = image[y:y+h, x:x+w].copy()
    if region.size == 0:
        return None
    return preprocess_image_for_tflite(region, target_size=TARGET_SIZE)

def is_valid_box(box, min_size=MIN_BOX_SIZE):
    """Checks if the box is valid based on the minimum size constraints."""
    x, y, w, h = box
    return w >= min_size and h >= min_size

def filter_top_k_boxes(boxes, k=TOP_K):
    sorted_boxes = sorted(boxes, key=lambda b: b[2]*b[3], reverse=True)
    return sorted_boxes[:k]




