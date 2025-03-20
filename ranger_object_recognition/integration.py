# integration.py

# Main API function for object detection

import glob
import os
import argparse
import time
import concurrent.futures
import math
from ranger_object_recognition import config, utils, feature_extractor, matching
import cv2
import numpy as np
from detect import run_detection

def find_item_in_scene():
    # Set paths and thresholds
    # yolo_weights = "best-fp16.tflite" 
    # source = "test_images/keys_scene/keys_scene5.jpeg"  # Scene images directory (or a single image)
    # data_yaml = "data.yaml"
    # feature_model_path = "mobilenet-v3-tensorflow2-small-075-224-feature-vector-v1"  
    # reference_dir = "test_images/keys_ref"  # Directory with reference images
    # similarity_threshold = 0.3
    valid_crops_dir = "valid_crops"  # Directory to save valid crops
    if not os.path.exists(valid_crops_dir):
        os.makedirs(valid_crops_dir)
    invalid_crops_dir = "invalid_crops"  # Directory to save invalid crops for debugging
    if not os.path.exists(invalid_crops_dir):
        os.makedirs(invalid_crops_dir)

    # 1. Run detection to get cropped regions (and optionally detection info)
    cropped_regions = run_detection(
        weights=config.YOLO_MODEL_PATH, source=config.SCENE_IMAGE_PATH, data=config.DATA_YAML_PATH, imgsz=(640, 640),
        conf_thres=config.YOLO_SIMILARITY_THRESHOLD, iou_thres=0.45, max_det=1000, device="",
        view_img=False, save_txt=False, save_format=0, save_csv=False, save_conf=False,
        save_crop=True, nosave=True, classes=None, agnostic_nms=False,
        augment=False, visualize=False, update=False, project="runs/detect",
        name="exp", exist_ok=False, line_thickness=3, hide_labels=False,
        hide_conf=False, half=False, dnn=False, vid_stride=1
    )
    print(f"Collected {len(cropped_regions)} cropped regions from detection.")

    # 2. Load the feature extraction model
    feature_model = feature_extractor.load_feature_extractor(config.FEATURE_MODEL_PATH)

    # 3. Process each cropped region: Preprocess and extract features
    crop_features = []
    for idx, crop in enumerate(cropped_regions):
        crop_input = feature_extractor.preprocess_crop(crop, target_size=(224, 224))
        vector = feature_extractor.extract_feature_vector(feature_model, crop_input)
        crop_features.append(vector)
        print(f"Extracted feature vector for crop {idx}.")

    # 4. Load reference images and extract their feature vectors
    ref_features = feature_extractor.load_reference_features(feature_model, config.REFERENCE_IMAGE_DIRECTORY, target_size=(224, 224))

    # 5. Compare each crop against the reference images using cosine similarity
    for idx, crop_vector in enumerate(crop_features):
        max_sim = 0
        for ref_path, ref_vector in ref_features.items():
            sim = matching.cosine_similarity(crop_vector, ref_vector)
            max_sim = max(max_sim, sim)
        print(f"Crop {idx} max similarity: {max_sim:.2f}")
        if max_sim >= config.FEATURE_SIMILARITY_THRESHOLD:
            print(f"Crop {idx} is considered valid (similarity {max_sim:.2f}).")
            # Save the valid crop to disk
            save_path = os.path.join(valid_crops_dir, f"crop_{idx}.jpg")
            cv2.imwrite(save_path, cropped_regions[idx])
            print(f"Saved valid crop {idx} to {save_path}")
        else:
            print(f"Crop {idx} is filtered out (similarity {max_sim:.2f}).")
            # Save the invalid crop to disk for debugging
            save_path = os.path.join(invalid_crops_dir, f"crop_{idx}.jpg")
            cv2.imwrite(save_path, cropped_regions[idx])
            print(f"Saved invalid crop {idx} to {save_path}")


if __name__ == "__main__":
    find_item_in_scene()