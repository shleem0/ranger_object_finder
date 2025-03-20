# config.py

import os

# Model configuration
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
YOLO_MODEL_PATH = os.path.join(BASE_DIR, "best-fp16.tflite")
FEATURE_MODEL_PATH = os.path.join(BASE_DIR, "mobilenet-v3-tensorflow2-small-075-224-feature-vector-v1")
TARGET_SIZE = (224, 224)
YOLO_SIMILARITY_THRESHOLD = 0.2 # Might have to adjust
FEATURE_SIMILARITY_THRESHOLD = 0.3 # Might have to adjust
DATA_YAML_PATH = os.path.join(BASE_DIR, "data.yaml")


# Example reference image paths (used if --ref_dir is not provided)
REFERENCE_IMAGE_DIRECTORY = "ranger_object_recognition/example images/keys_ref"
# Example scene image path (used if --scene is not provided)  
SCENE_IMAGE_PATH = "ranger_object_recognition/example images/keys_scene.jpeg"

# Multithreading config
NUM_WORKERS = 4
