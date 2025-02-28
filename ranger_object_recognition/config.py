# config.py

# Model configuration
MODEL_TFLITE_PATH = "ranger_object_recognition/feature_extractor_int8.tflite"
TARGET_SIZE = (224, 224)
SIMILARITY_THRESHOLD = 0.4 # Might have to adjust
MIN_BOX_SIZE = 20
DOWNSCALE_FACTOR = 0.25
TOP_K = 5000

# Reference and scene image paths
REFERENCE_IMAGE_PATHS = [
    "Keys/keys_ref1.jpeg",
    "Keys/keys_ref2.jpeg",
    "Keys/keys_ref3.jpeg",
    "Keys/keys_ref4.jpeg",
]
SCENE_IMAGE_PATH = "Keys/keys_scene2.jpeg"

# Multithreading config
NUM_WORKERS = 4