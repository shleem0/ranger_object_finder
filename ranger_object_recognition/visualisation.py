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

# Initial parameters for vertical axis (in cm)
ruler_position_top = 50    # Topmost ruler mark (in cm)
ruler_position_bottom = 10  # Bottommost ruler mark (in cm)

# ---------------------------
# Vertical axis functions
# ---------------------------
def pixels_to_cm(y_pixel, height):
    """
    Map y_pixel (0 at bottom, height at top) to cm range [ruler_position_bottom, ruler_position_top].
    """
    # When y_pixel=0, we get top (50 cm), and when y_pixel=height, we get bottom (10 cm).
    return ruler_position_bottom + (ruler_position_top - ruler_position_bottom) * (height - y_pixel) / float(height)

def ruler_spacing_at_position(y_pixel, height):
    """
    Calculate spacing between ticks for the vertical axis.
    Larger at the bottom, smaller at the top.
    """
    max_spacing = 50
    min_spacing = 10
    # The fraction grows as y_pixel increases; use it to linearly scale the spacing.
    fraction = y_pixel / float(height)
    spacing = max_spacing * fraction
    if spacing < min_spacing:
        spacing = min_spacing
    return spacing

# ---------------------------
# Horizontal axis functions
# ---------------------------
# We want the horizontal axis at y_center = height//2,
# mapping x in [0, width] -> cm in [-14, +14].
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

# ---------------------------
# Mouse click event: print coordinates using the rulers
# ---------------------------
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Compute horizontal coordinate in cm using the horizontal ruler mapping
        horizontal_cm = x_pixel_to_cm(center_x, param["width"])
        # Compute vertical coordinate in cm using the vertical ruler mapping
        vertical_cm = pixels_to_cm(center_y, param["height"])
        print(f"Clicked at (pixels): X = {x} px, Y = {y} px")
        print(f"Coordinates from rulers: X = {vertical_cm:.2f} cm, Y =  {horizontal_cm:.2f} cm\n")

def draw_bounding_boxes(scene_img, matches):
    """
    Draws only the bounding boxes for verified candidates on the scene image.
    :param scene_img: Original scene image (BGR format).
    :param matches: List of tuples (box, similarity), where each box is [x1, y1, x2, y2, score, class].
    :return: Annotated image with bounding boxes.
    """
    img_copy = scene_img.copy()
    for (box, sim) in matches:
        # Unpack bounding box coordinates; ignore similarity for drawing if not needed
        x1, y1, x2, y2, _, _ = map(int, box[:4])
        cv2.rectangle(img_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # Optionally, you can add text (e.g., similarity) if desired:
        # cv2.putText(img_copy, f"{sim:.2f}", (x1, y1 - 10),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return img_copy