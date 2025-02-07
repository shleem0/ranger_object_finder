import os
import tensorflow as tf
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
from numpy import dot
from numpy.linalg import norm

# ----- CONFIGURATION DEFAULTS -----
DEFAULT_MODEL_PATH = "ranger_object_recognition/._mobilenet-v3-tensorflow2-small-075-224-feature-vector-v1"
DEFAULT_SIMILARITY_THRESHOLD = 0.5  # Adjust if needed (may be too high)
DEFAULT_TARGET_SIZE = (224, 224)
DEFAULT_MIN_BOX_SIZE = 20
DEFAULT_DOWNSCALE_FACTOR = 0.25
DEFAULT_TOP_K = 1000
# ----- HELPER FUNCTIONS -----

def load_feature_extractor(model_path):
    abs_path = os.path.abspath(model_path)
    model = tf.saved_model.load(abs_path)
    print("Model loaded successfully from:", abs_path)
    return model

def preprocess_image_for_model(img, target_size=DEFAULT_TARGET_SIZE):
    img_resized = cv2.resize(img, target_size)
    img_normalized = img_resized.astype(np.float32) / 255.0
    return np.expand_dims(img_normalized, axis=0)

def load_and_preprocess_image(image_path, scale=1.0):
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Could not load image at {image_path}")
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    if scale != 1.0:
        height, width = img.shape[:2]
        img = cv2.resize(img, (int(width * scale), int(height * scale)))
    return img

def cosine_similarity(vec1, vec2):
    return dot(vec1, vec2) / (norm(vec1) * norm(vec2) + 1e-10)

def get_candidate_regions(image):
    ss = cv2.ximgproc.segmentation.createSelectiveSearchSegmentation()
    ss.setBaseImage(image)
    ss.switchToSelectiveSearchFast()
    rects = ss.process()
    print(f"Generated {len(rects)} candidate regions.")
    return rects

def is_valid_box(box, min_size=DEFAULT_MIN_BOX_SIZE):
    x, y, w, h = box
    return w >= min_size and h >= min_size

def filter_top_k_boxes(boxes, k=DEFAULT_TOP_K):
    sorted_boxes = sorted(boxes, key=lambda b: b[2]*b[3], reverse=True)
    return sorted_boxes[:k]

def extract_region(image, box):
    if not is_valid_box(box):
        return None
    x, y, w, h = box
    region = image[y:y+h, x:x+w]
    if region.size == 0:
        return None
    return preprocess_image_for_model(region)

def extract_feature_vector(model, image_input):
    features = model(image_input)
    return features.numpy().flatten()

def visualize_matches(scene_img, matches):
    img_to_show = scene_img.copy()
    for (box, sim) in matches:
        x, y, w, h = box
        cv2.rectangle(img_to_show, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.putText(img_to_show, f"{sim:.2f}", (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    plt.figure(figsize=(10, 10))
    plt.imshow(img_to_show)
    plt.axis('off')
    plt.title("Matched Candidate Regions")
    plt.show()

# ----- MAIN API FUNCTION -----
def find_item_in_scene(scene_image_path, reference_image_paths,
                       model_path=DEFAULT_MODEL_PATH,
                       similarity_threshold=DEFAULT_SIMILARITY_THRESHOLD,
                       downscale_factor=DEFAULT_DOWNSCALE_FACTOR):
    """
    Given a scene image path and a list of reference image paths, this function:
      - Loads the feature extractor model.
      - Extracts feature vectors for all reference images.
      - Runs selective search on the scene image (downscaled by downscale_factor).
      - Filters candidate regions and extracts feature vectors.
      - Compares each candidate to the reference vectors (using max cosine similarity).
      - Returns a list of matches (bounding boxes and similarity scores).
    """
    start_time = time.time()
    
    model = load_feature_extractor(model_path)
    
    # Process reference images
    ref_vectors = []
    for ref_path in reference_image_paths:
        ref_img = load_and_preprocess_image(ref_path)
        ref_input = preprocess_image_for_model(ref_img)
        ref_vector = extract_feature_vector(model, ref_input)
        ref_vectors.append(ref_vector)
    print(f"Extracted {len(ref_vectors)} reference feature vectors.")
    
    # Process scene image
    scene_img = load_and_preprocess_image(scene_image_path, scale=downscale_factor)
    
    candidate_boxes = get_candidate_regions(scene_img)
    candidate_boxes = [box for box in candidate_boxes if is_valid_box(box)]
    candidate_boxes = filter_top_k_boxes(candidate_boxes)
    
    print(f"Candidate regions after filtering: {len(candidate_boxes)}")
    
    matches = []
    processed = 0
    for box in candidate_boxes:
        region_input = extract_region(scene_img, box)
        if region_input is None:
            continue
        region_vector = extract_feature_vector(model, region_input)
        sims = [cosine_similarity(ref_vector, region_vector) for ref_vector in ref_vectors]
        max_sim = max(sims)
        if max_sim >= similarity_threshold:
            matches.append((box, max_sim))
        processed += 1
        if processed % 500 == 0:
            print(f"Processed {processed} regions...")
    
    print(f"Found {len(matches)} candidate matches above threshold {similarity_threshold}.")
    print(f"Total processing time: {time.time() - start_time:.2f} seconds")
    
    # Visualising matches for debugging
    visualize_matches(scene_img, matches)
    
    return matches