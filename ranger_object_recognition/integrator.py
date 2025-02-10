import time
from ranger_object_recognition import config, utils, proposals, feature_extractor, matching, visualization

def find_item_in_scene(scene_image_path, reference_image_paths,
                       model_path=config.DEFAULT_MODEL_PATH,
                       similarity_threshold=config.DEFAULT_SIMILARITY_THRESHOLD,
                       downscale_factor=config.DEFAULT_DOWNSCALE_FACTOR):
    """
    Given a scene image path and a list of reference image paths, this function:
      - Loads the feature extractor model.
      - Extracts feature vectors for all reference images.
      - Runs selective search on the scene image (downscaled by downscale_factor).
      - Filters candidate regions and extracts feature vectors.
      - Compares each candidate to the reference vectors (using max cosine similarity).
      - Applies NMS and returns a list of matches (bounding boxes and similarity scores).
    """
    start_time = time.time()
    
    model = feature_extractor.load_feature_extractor(model_path)
    
    # Process reference images
    ref_vectors = []
    for ref_path in reference_image_paths:
        ref_img = utils.load_and_preprocess_image(ref_path)
        ref_input = utils.preprocess_image_for_model(ref_img, target_size=config.DEFAULT_TARGET_SIZE)
        ref_vector = feature_extractor.extract_feature_vector(model, ref_input)
        ref_vectors.append(ref_vector)
    print(f"Extracted {len(ref_vectors)} reference feature vectors.")
    
    # Process scene image
    scene_img = utils.load_and_preprocess_image(scene_image_path, scale=downscale_factor)
    
    candidate_boxes = proposals.get_candidate_regions(scene_img)
    candidate_boxes = [box for box in candidate_boxes if proposals.is_valid_box(box)]
    candidate_boxes = proposals.filter_top_k_boxes(candidate_boxes)
    print(f"Candidate regions after filtering: {len(candidate_boxes)}")
    
    matches = []
    processed = 0
    for box in candidate_boxes:
        # Crop the region from the scene image using the box coordinates
        x, y, w, h = box
        region = scene_img[y:y+h, x:x+w]
        region_input = utils.preprocess_image_for_model(region, target_size=config.DEFAULT_TARGET_SIZE)
        if region_input is None:
            continue
        region_vector = feature_extractor.extract_feature_vector(model, region_input)
        sims = [matching.cosine_similarity(ref_vector, region_vector) for ref_vector in ref_vectors]
        max_sim = max(sims)
        if max_sim >= similarity_threshold:
            matches.append((box, max_sim))
        processed += 1
        if processed % 500 == 0:
            print(f"Processed {processed} regions...")
    
    print(f"Found {len(matches)} candidate matches above threshold {similarity_threshold}.")
    
    # Apply non-maximum suppression to reduce overlapping boxes
    matches = matching.apply_nms(matches, iou_threshold=0.5)
    print(f"Matches after NMS: {len(matches)}")
    
    print(f"Total processing time: {time.time() - start_time:.2f} seconds")
    
    # Visualize matches for debugging
    visualization.visualize_matches(scene_img, matches)
    
    return matches

if __name__ == "__main__":
    scene_path = "path_to_your_scene_image.jpg"
    ref_paths = ["path_to_reference1.jpg", "path_to_reference2.jpg", "path_to_reference3.jpg"]
    matches = find_item_in_scene(scene_path, ref_paths)
    print("Matches:", matches)