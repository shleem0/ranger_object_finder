# matching.py

# Code for calculating cosine similarity and applying non-maximum suppression

import numpy as np
from numpy import dot
from numpy.linalg import norm
import cv2

def cosine_similarity(vec1, vec2):
    """Computes cosine similarity between two vectors."""
    return dot(vec1, vec2) / (norm(vec1) * norm(vec2) + 1e-10)

def apply_nms(matches, iou_threshold=0):
    """
    Applies non-maximum suppression to a list of candidate matches.
    Each match is a tuple (box, score). Returns a filtered list of matches.
    """
    if not matches:
        return []
    boxes = [match[0] for match in matches]
    scores = [match[1] for match in matches]
    boxes_array = np.array(boxes).tolist()
    indices = cv2.dnn.NMSBoxes(boxes_array, scores, score_threshold=0.0, nms_threshold=iou_threshold)
    indices = [i[0] if isinstance(i, (list, tuple, np.ndarray)) else i for i in indices]
    return [matches[i] for i in indices]

