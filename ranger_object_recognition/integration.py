# integration.py

# Main API function for object detection

import glob
import os
import argparse
import time
import concurrent.futures
import math
from ranger_object_recognition import config, utils, proposals, feature_extractor, matching, visualisation

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
    args = parser.parse_args()

    # Overriding default paths if provided via command line
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
    
    # visualisation.visualise_matches(scene_img, matches)
    # return matches
    
    print(matches)

if __name__ == "__main__":
    find_item_in_scene()