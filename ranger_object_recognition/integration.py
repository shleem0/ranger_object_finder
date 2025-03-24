# integration.py

# Main API function for object detection
import sys
import glob
import os
import argparse
import time
from ranger_object_recognition import config, or_utils, feature_extractor, matching
import cv2
import numpy as np
from .detect import run_detection
import time
from . import measurements

def find_item_in_scene(scene_path, visualise = False):
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
    or_utils.clear_directory(valid_crops_dir)
    or_utils.clear_directory(invalid_crops_dir)
    config.SCENE_IMAGE_PATH = scene_path

        # Parsing command line arguments
    # parser = argparse.ArgumentParser(description="Run object recognition pipeline")
    # parser.add_argument("--ref_dir", type=str, help="Directory containing reference images")
    # parser.add_argument("--model", type=str, help="Path to TFLite model")
    # parser.add_argument("--visualise", action="store_true", help="Visualise the annotated image")
    # args = parser.parse_args()


    # if __name__ != "__main__":
    #     args = argparse.Namespace()
    #     args.visualise = False
    # Use YOLO model to detect potential items in the scene
    start_time = time.time()
    cropped_regions, boxes_list = run_detection(
        weights=config.YOLO_MODEL_PATH, source=config.SCENE_IMAGE_PATH, data=config.DATA_YAML_PATH, imgsz=(640, 640),
        conf_thres=config.YOLO_SIMILARITY_THRESHOLD, iou_thres=0.45, max_det=1000, device="",
        view_img=False, save_txt=False, save_format=0, save_csv=False, save_conf=False,
        save_crop=False, nosave=True, classes=None, agnostic_nms=False,
        augment=False, visualize=False, update=False, project="runs/detect",
        name="exp", exist_ok=False, line_thickness=3, hide_labels=False,
        hide_conf=False, half=False, dnn=False, vid_stride=1
    )
    print(f"Collected {len(cropped_regions)} cropped regions from detection.", file=sys.stderr)

    # Load the feature extraction model
    feature_model = feature_extractor.load_feature_extractor(config.FEATURE_MODEL_PATH)
    # start_time = time.time()
    # Process each cropped region from YOLO: Preprocess and extract features
    crop_features = []
    for idx, crop in enumerate(cropped_regions):
        crop_input = feature_extractor.preprocess_crop(crop, target_size=(224, 224))
        if crop_input is None:
            print(f"Skipping crop {idx} due to empty or invalid crop.", file=sys.stderr)
            continue
        vector = feature_extractor.extract_feature_vector(feature_model, crop_input)
        crop_features.append(vector)
        print(f"Extracted feature vector for crop {idx}.", file=sys.stderr)

    # Load reference images and extract features
    ref_features = feature_extractor.load_reference_features(feature_model, config.REFERENCE_IMAGE_DIRECTORY, target_size=(224, 224))

    # Compare each crop feature to the reference features
    valid_indices = []
    for idx, crop_vector in enumerate(crop_features):
        max_sim = 0
        for ref_path, ref_vector in ref_features.items():
            sim = matching.cosine_similarity(crop_vector, ref_vector)
            max_sim = max(max_sim, sim)
        # print(f"Crop {idx} max similarity: {max_sim:.2f}", file=sys.stderr)
        if max_sim >= config.FEATURE_SIMILARITY_THRESHOLD:
            print(f"Crop {idx} is considered valid (similarity {max_sim:.2f}).", file=sys.stderr)
            valid_indices.append(idx)
            # Save the valid crop to disk
            save_path = os.path.join(valid_crops_dir, f"crop_{idx}.jpg")
            cv2.imwrite(save_path, cropped_regions[idx])
            print(f"Saved valid crop {idx} to {save_path}")
        else:
            print(f"Crop {idx} is filtered out (similarity {max_sim:.2f}).", file=sys.stderr)
            # Save the invalid crop to disk for debugging
            save_path = os.path.join(invalid_crops_dir, f"crop_{idx}.jpg")
            cv2.imwrite(save_path, cropped_regions[idx])
            print(f"Saved invalid crop {idx} to {save_path}", file=sys.stderr)
    end_time = time.time()

    print("Bounding box coordinates for valid crops:", file=sys.stderr)
    for idx in valid_indices:
        print(boxes_list[idx])
    full_img = cv2.imread(config.SCENE_IMAGE_PATH)
    if full_img is not None:
        full_height = full_img.shape[0]
        print("Distance (cm) of valid detections (center of bounding box):", file=sys.stderr)
        for idx in valid_indices:
            box = boxes_list[idx]
            center_y = (box[1] + box[3]) / 2.0  # Calculate center y coordinate
            # Convert the center y coordinate (pixel value) to cm using measurements function
            distance_cm = measurements.pixels_to_cm(center_y, full_height)
            print(f"Crop {idx} center is at {distance_cm:.2f} cm from camera.", file=sys.stderr)
    else:
        print("Error: Could not load full scene image for measurement calculations.", file=sys.stderr)

    # Optional visualisation if you want to see the annotated image
    if visualise:
        if valid_indices:
            # Load the full scene image
            if full_img is None:
                print("Error: Could not load full scene image for annotation.", file=sys.stderr)
            else:
                # Create a copy for annotation
                annotated_img = full_img.copy()
                for idx in valid_indices:
                    box = boxes_list[idx]
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated_img, f"Crop {idx}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # Display the annotated image in a popup window
                cv2.imshow("Valid Detections on Full Scene", annotated_img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
        else:
            print("No valid detections to annotate on the full scene image.", file=sys.stderr)
    print(f"Processing time: {end_time - start_time:.2f} seconds.", file=sys.stderr)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run object recognition pipeline")
    parser.add_argument("--scene", type=str, help="Path to scene image")
    parser.add_argument("--ref_dir", type=str, help="Directory containing reference images")
    parser.add_argument("--model", type=str, help="Path to TFLite model")
    parser.add_argument("--visualise", action="store_true", help="Visualise the annotated image")
    args = parser.parse_args()
    

        # Overriding default paths if provided via command line
    if args.model:
        # Load tfLite model
        config.MODEL_TFLITE_PATH = args.model
        
    if args.ref_dir:
        # Use glob to find all image files in the directory
        config.REFERENCE_IMAGE_PATHS = glob.glob(os.path.join(args.ref_dir, "*.*"))
        if not config.REFERENCE_IMAGE_PATHS:
            print(f"Warning: No images found in {args.ref_dir}", file=sys.stderr)

    find_item_in_scene(args.scene, args.visualise)