# integration.py

# Main API function for object detection

import glob
import os
import argparse
import time
import concurrent.futures
import math
from ranger_object_recognition import config, utils, proposals, feature_extractor, matching, visualisation
import cv2
import numpy as np

def process_candidate_chunk(candidate_chunk, scene_img, ref_vectors):
    """
    Processes a chunk of candidate boxes in one thread.
    This function creates its own TFLite interpreter instance and processes each candidate region
    in the provided chunk, returning a list of matches that meet the similarity threshold.
    """
    # Create a new interpreter instance for this thread
    interpreter, input_details, output_details = feature_extractor.load_tflite_model(config.MODEL_TFLITE_PATH)
    chunk_matches = []
    
    for box in candidate_chunk:
        region_input = utils.extract_region(scene_img, box)
        if region_input is None:
            continue
        region_vector = feature_extractor.extract_feature_vector_tflite(interpreter, input_details, output_details, region_input)
        sims = [matching.cosine_similarity(ref_vector, region_vector) for ref_vector in ref_vectors]
        max_sim = max(sims)
        if max_sim >= config.SIMILARITY_THRESHOLD:
            chunk_matches.append((box, max_sim))
    
    return chunk_matches

def find_item_in_scene():
    start_time = time.time()

    # Parsing command line arguments
    parser = argparse.ArgumentParser(description="Run object recognition pipeline")
    parser.add_argument("--ref_dir", type=str, help="Directory containing reference images")
    parser.add_argument("--scene", type=str, help="Path to scene image")
    parser.add_argument("--model", type=str, help="Path to TFLite model")
    args = parser.parse_args()

    # Overriding default paths if provided via command line
    if args.model:
        # Load tfLite model
        config.MODEL_TFLITE_PATH = args.model

    if args.ref_dir:
        # Use glob to find all image files in the directory
        config.REFERENCE_IMAGE_PATHS = glob.glob(os.path.join(args.ref_dir, "*.*"))
        if not config.REFERENCE_IMAGE_PATHS:
            print(f"Warning: No images found in {args.ref_dir}")

    if args.scene:
        config.SCENE_IMAGE_PATH = args.scene
    # Load TFLite model once to process reference images
    base_interpreter, base_input_details, base_output_details = feature_extractor.load_tflite_model(config.MODEL_TFLITE_PATH)
    
    
    # Process reference images
    ref_vectors = []
    for ref_path in config.REFERENCE_IMAGE_PATHS:
        ref_img = utils.load_image_uint8(ref_path)
        ref_input = utils.preprocess_image_for_tflite(ref_img, config.TARGET_SIZE)
        ref_vector = feature_extractor.extract_feature_vector_tflite(base_interpreter, base_input_details, base_output_details, ref_input)
        ref_vectors.append(ref_vector)
    print(f"Extracted {len(ref_vectors)} reference feature vectors.")
    
    # Process scene image
    scene_img = utils.load_image_uint8(config.SCENE_IMAGE_PATH, scale=config.DOWNSCALE_FACTOR)
    
    # Generate candidate regions
    candidate_boxes = proposals.get_candidate_regions(scene_img)
    candidate_boxes = [box for box in candidate_boxes if utils.is_valid_box(box)]
    candidate_boxes = proposals.filter_top_k_boxes(candidate_boxes, k=config.TOP_K)
    print(f"Candidate regions after filtering: {len(candidate_boxes)}")
    
    # Partition candidate_boxes into chunks for parallel processing
    num_workers = config.NUM_WORKERS if hasattr(config, 'NUM_WORKERS') else 4
    chunk_size = math.ceil(len(candidate_boxes) / num_workers)
    candidate_chunks = [candidate_boxes[i:i + chunk_size] for i in range(0, len(candidate_boxes), chunk_size)]
    
    # Use ThreadPoolExecutor to process candidate chunks in parallel
    matches = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=num_workers) as executor:
        futures = [executor.submit(process_candidate_chunk, chunk, scene_img, ref_vectors) for chunk in candidate_chunks]
        for future in concurrent.futures.as_completed(futures):
            chunk_matches = future.result()
            matches.extend(chunk_matches)
    
    print(f"Found {len(matches)} candidate matches above threshold {config.SIMILARITY_THRESHOLD}.")
    
    # Apply Non-Maximum Suppression (NMS)
    matches = matching.apply_nms(matches, iou_threshold=0)
    print(f"Matches after NMS: {len(matches)}")
    print(f"Total processing time: {time.time() - start_time:.2f} seconds")
    
    # --- New Visualization Variant: Drawing rulers on the scene image ---
    
    # Initial parameters for vertical axis (in cm)
    ruler_position_top = 50    # Topmost ruler mark (in cm)
    ruler_position_bottom = 10  # Bottommost ruler mark (in cm)

    # Vertical axis functions

    def pixels_to_cm(y_pixel, height):
        """
        Map y_pixel (0 at bottom, height at top) to cm range [ruler_position_bottom, ruler_position_top].
        """
        return ruler_position_bottom + (ruler_position_top - ruler_position_bottom) * (height - y_pixel) / float(height)


    def ruler_spacing_at_position(y_pixel, height):
        """
        Calculate spacing between ticks for the vertical axis.
        Larger at the bottom, smaller at the top.
        """
        max_spacing = 50
        min_spacing = 10
        fraction = y_pixel / float(height)
        spacing = max_spacing * fraction
        if spacing < min_spacing:
            spacing = min_spacing
        return spacing

    # Horizontal axis functions

    H_MIN_CM = -14
    H_MAX_CM = 14

    def x_pixel_to_cm(x_pixel, width):
        """
        Map x_pixel in [0, width] -> cm in [H_MIN_CM, H_MAX_CM].
        The center of the image (x = width/2) corresponds to 0 cm.
        """
        return H_MIN_CM + (H_MAX_CM - H_MIN_CM) * (x_pixel / float(width))


    def x_cm_to_pixel(x_cm, width):
        """
        Inverse: map cm in [H_MIN_CM, H_MAX_CM] -> x_pixel in [0, width].
        """
        return int((x_cm - H_MIN_CM) / (H_MAX_CM - H_MIN_CM) * width)

    # Mouse click event: print coordinates using the rulers

    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            height = param["height"]
            width = param["width"]
            horizontal_cm = x_pixel_to_cm(x, width)
            vertical_cm = pixels_to_cm(y, height)
            print(f"Clicked at (pixels): X = {x} px, Y = {y} px")
            print(f"Coordinates from rulers: X = {horizontal_cm:.2f} cm, Y = {vertical_cm:.2f} cm\n")

    # Function to process and display the image with rulers

    def processImg(image, matches):
        frame = image.copy()
        height, width, _ = frame.shape

        # 1) Draw the vertical ruler on the left
        y_position = 0
        while y_position < height:
            cm_value = pixels_to_cm(y_position, height)
            spacing = ruler_spacing_at_position(y_position, height)
            y_position_int = int(y_position)
            cv2.line(frame, (10, y_position_int), (30, y_position_int), (0, 255, 0), 2)
            cv2.putText(frame, f"{cm_value:.1f} cm", (35, y_position_int + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            y_position += spacing

        # 2) Draw the horizontal ruler across the middle
        y_center = height // 2
        for cm_h in np.arange(H_MIN_CM, H_MAX_CM + 0.1, 2.0):
            x_pos = x_cm_to_pixel(cm_h, width)
            cv2.line(frame, (x_pos, y_center - 10), (x_pos, y_center + 10), (0, 0, 255), 2)
            cv2.putText(frame, f"{cm_h:.1f}", (x_pos - 10, y_center - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 3) Draw bounding boxes for matches
        for (box, sim) in matches:
            # Each box: (x, y, w, h)
            x, y, w, h = box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

            # Calculate centre of bounding box in pixels
            center_x_px = x + w // 2
            center_y_px = y + h // 2

            # Convert centre to cm
            center_x_cm = x_pixel_to_cm(center_x_px, width)
            center_y_cm = pixels_to_cm(center_y_px, height)

            # Draw a small circle at the centre
            cv2.circle(frame, (center_x_px, center_y_px), 4, (255, 0, 0), -1)

            # Print text
            print(f"Bounding box center (pixels): ({center_x_px}, {center_y_px})")
            print(f"Bounding box center (cm): ({center_x_cm:.1f}, {center_y_cm:.1f})")

        # 4) Set up mouse callback for the ruler-based coords
        window_name = "Camera Feed with Rulers"
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, click_event, param={"width": width, "height": height})

        return frame

    # Display the scene image with both the ruler overlay and bounding boxes until a key is pressed
    processed_img = processImg(scene_img, matches)
    cv2.imshow("Camera Feed with Rulers", processed_img)
    cv2.waitKey(0)  # Wait indefinitely until a key is pressed
    cv2.destroyAllWindows()

    print(matches)  # Prints list of matches as tuples and similarities

if __name__ == "__main__":
    find_item_in_scene()