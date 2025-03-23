# Ranger Object Finder

A simple pipeline for detecting specific objects in a scene image by comparing them to reference images.

## How It Works

1. **Reference Images**:  
   Take reference photos of your target item (e.g., keys) from multiple angles through the Ranger™ mobile app

2. **Scene Analysis**:  
   Capture a scene photo where the object might be located.

3. **Object Detection (YOLOv5)**:  
   Use a YOLOv5 TFLite model to detect candidate regions in the scene image.

4. **Feature Extraction**:  
   Each detected region is passed through a MobileNetV3-based feature extraction model to obtain a feature vector.

5. **Similarity Matching**:  
   Compare each candidate feature vector with the reference feature vectors using cosine similarity. Regions exceeding a set similarity threshold are marked as valid matches.

6. **Results**:  
   Valid matches are saved and visually confirmed via bounding boxes drawn on the original scene image.

## Usage

To run the object recognition pipeline with default settings:

```bash
python -m ranger_object_recognition.integration
```

### Arguments
- `--scene` - Provide filepath of a scene image to run object detection on. Default in [example images](example%20images/keys_scene.jpeg)
- `--ref_dir` - Provide filepath of a directory containing the reference images taken on the mobile app. Default in [example images](example%20images/keys_ref)
- `--feat_model` - Provide filepath of the tensorflow feature vector model if needed
- `--yolo_model` - Provide filepath of the YOLOv5 TFLite model if needed
- `--visualise` - Optional visualisation of any detected items in the scene image post-inference

## Running live with queue and camera

```bash
python -m ranger_object_recognition.queue_worker --queue "ranger_object_recognition/image_queue"
```

### Arguments
- `--queue` - Provide filepath to a directory to store the images for the queue
- `--interval` - Interval in seconds to take photos (default 10)
- `--max_files` - Maximum number of images to have in the queue at one time (default 10)