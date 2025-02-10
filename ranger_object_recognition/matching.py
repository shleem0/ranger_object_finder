import numpy as np
from numpy import dot
from numpy.linalg import norm
import cv2

def cosine_similarity(vec1, vec2):
    """
    Computes the cosine similarity between two vectors.
    """
    return dot(vec1, vec2) / (norm(vec1) * norm(vec2) + 1e-10)

def apply_nms(matches, iou_threshold=0.5):
    """
    Applies non-maximum suppression (NMS) to a list of matches.
    Each match is a tuple (box, score) where box is (x, y, w, h).
    """
    if len(matches) == 0:
        return []
    
    boxes = [match[0] for match in matches]
    scores = [match[1] for match in matches]
    boxes_array = np.array(boxes).tolist()
    indices = cv2.dnn.NMSBoxes(boxes_array, scores, score_threshold=0.0, nms_threshold=iou_threshold)
    indices = [i[0] if isinstance(i, (list, tuple, np.ndarray)) else i for i in indices]
    nms_matches = [matches[i] for i in indices]
    return nms_matches