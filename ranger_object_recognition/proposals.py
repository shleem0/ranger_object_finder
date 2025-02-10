import cv2
from ranger_object_recognition.config import DEFAULT_MIN_BOX_SIZE, DEFAULT_TOP_K

def get_candidate_regions(image):
    """
    Generate candidate regions for object detection using selective search.
    """
    ss = cv2.ximgproc.segmentation.createSelectiveSearchSegmentation()
    ss.setBaseImage(image)
    ss.switchToSelectiveSearchFast()
    rects = ss.process()
    print(f"Generated {len(rects)} candidate regions.")
    return rects

def is_valid_box(box, min_size=DEFAULT_MIN_BOX_SIZE):
    """
    Return true if box's height and width are at least min_size.
    """
    x, y, w, h = box
    return w >= min_size and h >= min_size

def filter_top_k_boxes(boxes, k=DEFAULT_TOP_K):
    """
    Sorts candidate boxes by area (w * h) in descending order and returns the top k.
    """
    sorted_boxes = sorted(boxes, key=lambda b: b[2] * b[3], reverse=True)
    return sorted_boxes[:k]