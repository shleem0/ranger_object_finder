import cv2
import numpy as np

def load_and_process_image(image_path, scale=1.0):
    """
    Load an image from the specified path then:
    - Convert it to RGB format
    - Resize it by the specified scale factor
    - Convert it to a NumPy array
    """
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Failed to load image from {image_path}")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    if scale != 1.0:
        height, width = img.shape[:2]
        img = cv2.resize(img, (int(width * scale), int(height * scale)))
    return img
def preprocess_image_for_model(img, target_size=(224, 224)):
    """
    Resize and normalize an image for a model input. Normalize pixel values to [0, 1] and add a batch dimension.
    """
    img_resized = cv2.resize(img, target_size)
    img_normalized = img_resized.astype(np.float32) / 255.0
    return np.expand_dims(img_normalized, axis=0)