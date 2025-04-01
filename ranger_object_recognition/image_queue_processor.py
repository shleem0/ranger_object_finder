import cv2
import os
import sys
import glob
import argparse
import time  # ensure time is imported
from .integration import find_item_in_scene

def process_queue(queue_dir, max_files=10, stop_event = None):
    """
    Continuously processes up to max_files images from the queue directory one at a time.
    When the queue is empty, waits for 10 seconds before refreshing the list.
    """
    # Clear image_queue before running
    for file in os.listdir(queue_dir):
        os.remove(os.path.join(queue_dir, file))
    # Give time for camera to start up and take first photo
    time.sleep(6)
    while True:
        # Check if the stop event is set
        if stop_event and stop_event.is_set():
            print("Stop event set. Exiting queue processing.", file=sys.stderr)
            return
        # Supported image extensions
        supported_exts = ('.jpg', '.jpeg', '.png', '.bmp', '.gif')
        # List all image files in the queue directory
        queue_files = [os.path.join(queue_dir, f) for f in os.listdir(queue_dir) if f.lower().endswith(supported_exts)]
        queue_files.sort()  # sort files alphabetically
        if not queue_files:
            print("Queue empty, waiting...", file=sys.stderr)
            time.sleep(10)
            continue
        count = 0
        for file_path in queue_files:
            if count >= max_files:
                break
            print(f"Processing image: {file_path}", file=sys.stderr)
            # Call find_item_in_scene and capture its return values.
            # Assumes find_item_in_scene returns (cropped_regions, boxes_list, valid_indices)
            cropped_regions, boxes_list, valid_indices = find_item_in_scene(file_path, visualise=True) # Can set to true for debugging
            if valid_indices:
                print("Valid crop found. Stopping queue processing.", file=sys.stderr)
                if stop_event:
                    stop_event.set()
                return  # Exit the process_queue function entirely
            os.remove(file_path)
            print(f"Removed processed image: {file_path}", file=sys.stderr)
            count += 1
        # Short pause between batches
        time.sleep(2)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process a queue of scene images for object detection")
    parser.add_argument("--queue", type=str, required=True, help="Path to the directory containing queued images")
    args = parser.parse_args()
    process_queue(args.queue)