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
from .arduino_connection import send_coordinates

REF_FEATURES = None
FEATURE_MODEL = None

def get_reference_features(feature_model, target_size=(224, 224)):
    global REF_FEATURES
    if REF_FEATURES is None:
        REF_FEATURES = feature_extractor.load_reference_features(feature_model, config.REFERENCE_IMAGE_DIRECTORY, target_size=target_size)
        print(f"Extracted features from {len(REF_FEATURES)} reference images.")
    return REF_FEATURES

def get_feature_model():
    global FEATURE_MODEL
    if FEATURE_MODEL is None:
        FEATURE_MODEL = feature_extractor.load_feature_extractor(config.FEATURE_MODEL_PATH)
        print("Feature model loaded.")
    return FEATURE_MODEL

def find_item_in_scene(scene_path, visualise = False):
    # Set paths and thresholds
    valid_crops_dir = config.VALID_CROP_DIR  # Directory to save valid crops
    if not os.path.exists(valid_crops_dir):
        os.makedirs(valid_crops_dir)
    invalid_crops_dir = config.INVALID_CROP_DIR  # Directory to save invalid crops for debugging
    if not os.path.exists(invalid_crops_dir):
        os.makedirs(invalid_crops_dir)
    or_utils.clear_directory(valid_crops_dir)
    or_utils.clear_directory(invalid_crops_dir)
    config.SCENE_IMAGE_PATH = scene_path

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
    feature_model = get_feature_model()
    # start_time = time.time()
    # Process each cropped region from YOLO: Preprocess and extract features
    valid_cropped_regions = []
    valid_boxes_list = []
    crop_features = []
    for idx, crop in enumerate(cropped_regions):
        crop_input = feature_extractor.preprocess_crop(crop, target_size=(224, 224))
        if crop_input is None:
            print(f"Skipping crop {idx} due to empty or invalid crop.", file=sys.stderr)
            continue
        vector = feature_extractor.extract_feature_vector(feature_model, crop_input)
        crop_features.append(vector)
        valid_cropped_regions.append(crop)
        valid_boxes_list.append(boxes_list[idx])
        print(f"Extracted feature vector for crop {idx}.", file=sys.stderr)
    # Load reference images and extract features
    ref_features = get_reference_features(feature_model, target_size=(224, 224))
    # Compare each crop feature to the reference features and select the best valid crop
    best_idx = None
    best_sim = -1
    for idx, crop_vector in enumerate(crop_features):
        max_sim = 0
        for ref_path, ref_vector in ref_features.items():
            sim = matching.cosine_similarity(crop_vector, ref_vector)
            max_sim = max(max_sim, sim)
        print(f"Crop {idx} max similarity: {max_sim:.2f}", file=sys.stderr)
        if max_sim >= config.FEATURE_SIMILARITY_THRESHOLD and max_sim > best_sim:
            best_sim = max_sim
            best_idx = idx

    if best_idx is not None:
        print(f"Crop {best_idx} is considered the best valid crop (similarity {best_sim:.2f}).", file=sys.stderr)
        # Save the best valid crop to disk
        save_path = os.path.join(valid_crops_dir, f"crop_best.jpg")
        cv2.imwrite(save_path, valid_cropped_regions[best_idx])
        print(f"Saved best valid crop {best_idx} to {save_path}", file=sys.stderr)
        # Only keep this best crop for subsequent measurement and annotation steps
        valid_indices = [best_idx]
    else:
        print("No valid crops found.", file=sys.stderr)
        valid_indices = []
    end_time = time.time()
    
    print("Bounding box coordinates for valid crops:", file=sys.stderr)
    for idx in valid_indices:
        print(boxes_list[idx])
    full_img = cv2.imread(config.SCENE_IMAGE_PATH)
    if full_img is not None:
        full_height, full_width = full_img.shape[:2]
        print("Distance (cm) of valid detections (center of bounding box):", file=sys.stderr)
        for idx in valid_indices:
            box = valid_boxes_list[idx]  # box = [x1, y1, x2, y2]
            center_x = (box[0] + box[2]) / 2.0
            center_y = (box[1] + box[3]) / 2.0
            # Convert x and y pixel coordinates to cm using your measurements functions:
            distance_x_cm = measurements.x_pixel_to_cm(center_x, full_width)
            distance_y_cm = measurements.pixels_to_cm(center_y, full_height)
            print(f"Crop {idx} center is at (X: {distance_x_cm:.2f} cm, Y: {distance_y_cm:.2f} cm) from camera.", file=sys.stderr)
            send_coordinates(distance_x_cm, distance_y_cm)
    else:
        print("Error: Could not load full scene image for measurement calculations.", file=sys.stderr)

    # Optional visualisation if you want to see the annotated image
    if visualise:
        if valid_indices:
            # Load the full scene image if not already loaded
            if full_img is None:
                full_img = cv2.imread(config.SCENE_IMAGE_PATH)
            if full_img is None:
                print("Error: Could not load full scene image for annotation.", file=sys.stderr)
            else:
                # Create a copy for annotation
                annotated_img = full_img.copy()
                # Draw vertical ruler on the left side
                height, width, _ = annotated_img.shape
                y = 0
                while y < height:
                    cm_value = measurements.pixels_to_cm(y, height)
                    spacing = measurements.ruler_spacing_at_position(y, height)
                    y_int = int(y)
                    # Make tick lines thicker and longer
                    cv2.line(annotated_img, (10, y_int), (50, y_int), (0, 255, 0), 3)
                    
                    # Create text label
                    text = f"{cm_value:.1f} cm"
                    font_scale = 1.0
                    thickness = 2
                    color = (0, 255, 0)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    
                    # Get text size to draw a background rectangle
                    (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)
                    rect_x1, rect_y1 = (55, y_int - text_h // 2 - 5)
                    rect_x2, rect_y2 = (55 + text_w + 10, y_int + text_h // 2 + 5)
                    

                    # Then draw the text on top
                    text_org = (rect_x1 + 5, y_int + text_h // 2 - 2)
                    cv2.putText(annotated_img, text, text_org, font, font_scale, color, thickness)
                    
                    y += spacing
                
                # Draw horizontal ruler along the bottom
                for cm_val in np.arange(measurements.H_MIN_CM, measurements.H_MAX_CM + 0.1, 2.0):
                    x_pixel = measurements.x_cm_to_pixel(cm_val, width)
                    # Thicker and longer tick marks
                    cv2.line(annotated_img, (x_pixel, height - 60), (x_pixel, height - 10), (0, 0, 255), 3)
                    
                    # Create text label
                    text = f"{cm_val:.1f}"
                    font_scale = 1.0
                    thickness = 2
                    color = (0, 0, 255)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    
                    # Get text size to draw a background rectangle
                    (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)
                    rect_x1, rect_y1 = (x_pixel - text_w // 2 - 5, height - 65 - text_h)
                    rect_x2, rect_y2 = (x_pixel + text_w // 2 + 5, height - 65)
                    
                    # Then draw the text on top
                    text_org = (rect_x1 + 5, rect_y2 - 5)
                    cv2.putText(annotated_img, text, text_org, font, font_scale, color, thickness)
                # # Display the annotated image with rulers
                # cv2.imshow("Valid Detections on Full Scene with Rulers", annotated_img)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
                # Instead saving the annotated image to disk
                save_path = os.path.join(valid_crops_dir, "annotated_scene.jpg")
                cv2.imwrite(save_path, annotated_img)
                print(f"Saved annotated image with rulers to {save_path}", file=sys.stderr)
        else:
            print("No valid detections to annotate on the full scene image.", file=sys.stderr)
    print(f"Processing time: {end_time - start_time:.2f} seconds.", file=sys.stderr)
    return valid_cropped_regions, valid_boxes_list, valid_indices

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