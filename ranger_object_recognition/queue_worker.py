import threading
import argparse
import time
from ranger_object_recognition.image_capture import capture_image
from ranger_object_recognition.image_queue_processor import process_queue

def main(queue_dir, capture_interval, max_files):
    print("Initialising object recognition system, please wait...")

    stop_event = threading.Event()

    input("Press Enter to start camera capture and queue processing...")

    capture_thread = threading.Thread(target=capture_image, args=(queue_dir, capture_interval, stop_event))
    process_thread = threading.Thread(target=process_queue, args=(queue_dir, max_files, stop_event))
    
    capture_thread.daemon = True
    process_thread.daemon = True
    
    capture_thread.start()
    process_thread.start()
    
    print("Camera capture and queue processing started. Press Ctrl+C to exit.")
    
    # Wait for stop_event to be set. This will return as soon as stop_event is set.
    stop_event.wait()
    
    capture_thread.join()
    process_thread.join()
    print("All threads stopped. Exiting program.")
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run camera capture and queue processing concurrently")
    parser.add_argument("--queue", type=str, required=True, help="Path to the directory for queued images")
    parser.add_argument("--interval", type=int, default=10, help="Time interval (in seconds) between photo captures")
    parser.add_argument("--max_files", type=int, default=10, help="Maximum number of files to process per batch")
    args = parser.parse_args()
    
    main(args.queue, args.interval, args.max_files)