import os
import cv2
import time

def capture_image(queue_dir, capture_interval = 10):
        # Create queue directory if it doesn't exist
    if not os.path.exists(queue_dir):
        os.makedirs(queue_dir)
    # Open camera, should be 0 for default
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Camera opened successfully. Starting to capture photos every {} seconds...".format(capture_interval))
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame.")
                break

            # Create a unique filename using the current timestamp and an optional counter
            timestamp = int(time.time())
            filename = f"photo_{timestamp}.jpg"
            filepath = os.path.join(queue_dir, filename)

            # Save the captured frame to the queue directory
            cv2.imwrite(filepath, frame)
            print(f"Saved photo to {filepath}")

            # Wait for the specified interval before capturing the next frame
            time.sleep(capture_interval)
    except KeyboardInterrupt:
        print("Capture stopped by user.")
    finally:
        cap.release()