import threading
import argparse
import time
from ranger_object_recognition.image_capture import capture_image
from ranger_object_recognition.image_queue_processor import process_queue

def main(queue_dir, capture_interval, max_files):
    # Create and start a thread for capturing images
    capture_thread = threading.Thread(target=capture_image, args=(queue_dir, capture_interval))
    # Create and start a thread for processing the queue
    process_thread = threading.Thread(target=process_queue, args=(queue_dir, max_files))
    
    # Set threads as daemon so they exit when the main program exits
    capture_thread.daemon = True
    process_thread.daemon = True
    
    capture_thread.start()
    process_thread.start()
    
    print("Camera capture and queue processing started. Press Ctrl+C to exit.")
    
    # Keep the main thread alive until interrupted
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run camera capture and queue processing concurrently")
    parser.add_argument("--queue", type=str, required=True, help="Path to the directory for queued images")
    parser.add_argument("--interval", type=int, default=10, help="Time interval (in seconds) between photo captures")
    parser.add_argument("--max_files", type=int, default=10, help="Maximum number of files to process per batch")
    args = parser.parse_args()
    
    main(args.queue, args.interval, args.max_files)