import cv2
import os
import sys
import glob
import argparse
import time  # ensure time is imported
from .integration import find_item_in_scene

def process_queue(queue_dir, max_files=10):
    """
    Continuously processes up to max_files images from the queue directory one at a time.
    When the queue is empty, waits for 10 seconds before refreshing the list.
    """
    while True:
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
            find_item_in_scene(file_path)
            os.remove(file_path)
            print(f"Removed processed image: {file_path}", file=sys.stderr)
            count += 1
        # Optional short pause between batches
        time.sleep(2)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process a queue of scene images for object detection")
    parser.add_argument("--queue", type=str, required=True, help="Path to the directory containing queued images")
    args = parser.parse_args()
    process_queue(args.queue)