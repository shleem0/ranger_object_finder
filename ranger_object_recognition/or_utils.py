# utils.py

# Utility functions for loading and preprocessing images
import cv2
import numpy as np
import os
import shutil


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

def load_image_webcam_uint8(image, scale = 1.0):
    """Loads an image from the given path, converts it to RGB, downsizes it if needed, and returns it in uint8 format."""
    img = image
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

def capture_camera_image():
    """Capture an image from the webcam when 's' is pressed."""
    cap = cv2.VideoCapture(1)  # 0 usually refers to the default webcam
    if not cap.isOpened():
        raise Exception("Cannot open webcam")
    
    print("Press 's' to capture the scene image, or 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        cv2.imshow('Live Feed - Press s to capture', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            # When 's' is pressed, capture the image
            scene_img = frame.copy()
            break
        elif key == ord('q'):
            scene_img = None
            break
    
    cap.release()
    cv2.destroyAllWindows()
    return scene_img

def encode_image_for_ros(image):
    """
    Encode image for ros
    """
    success, encoded_image = cv2.imencode('.jpg', image)
    if not success:
        raise ValueError("Failed to encode image")
    return encoded_image.tobytes()

def clear_directory(directory):
    """Remove all files and subdirectories in the given directory."""
    for filename in os.listdir(directory):
        file_path = os.path.join(directory, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print(f"Failed to delete {file_path}. Reason: {e}")





