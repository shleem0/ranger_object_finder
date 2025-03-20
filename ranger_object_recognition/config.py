# config.py

import os

# Model configuration
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_TFLITE_PATH = os.path.join(BASE_DIR, "feature_extractor_int8.tflite")
TARGET_SIZE = (224, 224)
SIMILARITY_THRESHOLD = 0.4 # Might have to adjust
MIN_BOX_SIZE = 20
DOWNSCALE_FACTOR = 0.25
TOP_K = 5000

# Example reference image paths (used if --ref_dir is not provided)
REFERENCE_IMAGE_PATHS = [
    "ranger_object_recognition/example images/keys_ref/keys_ref1.jpeg",
    "ranger_object_recognition/example images/keys_ref/keys_ref2.jpeg",
    "ranger_object_recognition/example images/keys_ref/keys_ref3.jpeg",
    "ranger_object_recognition/example images/keys_ref/keys_ref4.jpeg",
]
# Example scene image path (used if --scene is not provided)  
# SCENE_IMAGE_PATH = "ranger_object_recognition/example images/keys_scene.jpeg"
SCENE_IMAGE_PATH = "ranger_object_recognition/example images/keys_ruler.jpg"

# Multithreading config
NUM_WORKERS = 4
