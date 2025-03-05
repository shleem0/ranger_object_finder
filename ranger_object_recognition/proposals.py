# proposals.py

# Code for candidate region generation and filtering using selective search

import cv2
from ranger_object_recognition.config import MIN_BOX_SIZE, TOP_K

def get_candidate_regions(image):
    """Uses Selective Search to generate candidate regions (bounding boxes)."""
    ss = cv2.ximgproc.segmentation.createSelectiveSearchSegmentation()
    ss.setBaseImage(image)
    ss.switchToSelectiveSearchFast()
    rects = ss.process()
    print(f"Generated {len(rects)} candidate regions.")
    return rects

# def is_valid_box(box, min_size=MIN_BOX_SIZE):
#     """Checks if a bounding box meets the minimum size requirement."""
#     x, y, w, h = box
#     return w >= min_size and h >= min_size

def filter_top_k_boxes(boxes, k=TOP_K):
    """Sorts boxes by area (w*h) in descending order and returns the top k."""
    sorted_boxes = sorted(boxes, key=lambda b: b[2]*b[3], reverse=True)
    return sorted_boxes[:k]

