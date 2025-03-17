# visualisation.py

# Utility function for drawing bounding boxes and visualizing matches on images
import cv2
import matplotlib.pyplot as plt

def visualise_matches(scene_img, matches):
    """
    Draws bounding boxes and similarity scores on the scene image and displays the result.
    """
    img_to_show = scene_img.copy()
    for (box, sim) in matches:
        x, y, w, h = box
        cv2.rectangle(img_to_show, (x, y), (x+w, y+h), (255, 0, 0), 2)

        center_x = x + w // 2
        center_y = y + h // 2
        cv2.putText(img_to_show, f"{sim:.2f}", (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    plt.figure(figsize=(10, 10))
    plt.imshow(img_to_show)
    plt.axis('off')
    plt.title("Matched Candidate Regions")
    plt.show()
