import visualisation
import cv2
import numpy as np

# Camera setup
#cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Change index if needed

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
    # Compute horizontal coordinate in cm using the horizontal ruler mapping
    horizontal_cm = x_pixel_to_cm(visualisation.center_x, param["width"])
    # Compute vertical coordinate in cm using the vertical ruler mapping
    vertical_cm = pixels_to_cm(visualisation.center_y, param["height"])
    print(f"Clicked at (pixels): X = {x} px, Y = {y} px")
    print(f"Coordinates from rulers: X = {vertical_cm:.2f} cm, Y =  {horizontal_cm:.2f} cm\n")

"""
while True:
    #ret, frame = cap.read()
    # if not ret:
       # print("Error: Camera feed unavailable")
        #break

    height, width, _ = frame.shape

    # 1) Draw the vertical axis (left ruler)
    y_position = 0
    while y_position < height:
        cm_value = pixels_to_cm(y_position, height)
        spacing = ruler_spacing_at_position(y_position, height)
       
        y_position_int = int(y_position)
        
        # Draw a tick (horizontal line) for the vertical ruler
        cv2.line(frame, (10, y_position_int), (30, y_position_int), (0, 255, 0), 2)
        cv2.putText(frame, f"{cm_value:.1f} cm", (35, y_position_int + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
       
        y_position += spacing

    # 2) Draw the horizontal axis across the middle
    y_center = height // 2
    #cv2.line(frame, (0, y_center), (width, y_center), (0, 0, 255), 2)

    # Place tick marks along the horizontal axis every 2 cm from -14 to +14
    for cm_h in np.arange(H_MIN_CM, H_MAX_CM + 0.1, 2.0):
        x_pos = x_cm_to_pixel(cm_h, width)
        
        cv2.line(frame, (x_pos, y_center - 10), (x_pos, y_center + 10), (0, 0, 255), 2)
        cv2.putText(frame, f"{cm_h:.1f}", (x_pos - 10, y_center - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Show the frame with both rulers
    cv2.imshow("Camera Feed with Rulers", frame)"
    "

    # Pass width and height to the mouse callback via a dictionary
    cv2.setMouseCallback("Camera Feed with Rulers", click_event, param={"width": width, "height": height})"
    

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break"
    

cap.release()
cv2.destroyAllWindows()"
"""