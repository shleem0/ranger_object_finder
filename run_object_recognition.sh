#!/bin/bash
export PYTHONPATH="/Volumes/Kian_T7/ranger_object_finder"
python3 -m ranger_object_recognition.queue_worker --queue "ranger_object_recognition/image_queue" --interval 10 --max_files 10