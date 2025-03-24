#!/bin/bash
python3 -m ranger_object_recognition.queue_worker --queue "ranger_object_recognition/image_queue" --interval 10 --max_files 10
