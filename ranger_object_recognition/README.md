# Object Recognition

## How It Works

The object recognition system follows these steps:

### **1. Reference Image Processing**  
- Multiple reference images of the item (e.g., keys) are captured at different angles (via the app).  
- These images are processed through a [TFLite model](feature_extractor_int8.tflite), which extracts a feature vector representing the item's visual characteristics.

### **2. Scene Image Analysis**  
- The Ranger robot periodically captures a scene image while searching for the lost item.  
- Will probable downscale scene image to improve processing efficiency.

### **3. Generating Candidate Regions**  
- The scene image undergoes selective search to identify candidate regions that may contain the item.

### **4. Feature Extraction & Matching**  
- Each candidate region is cropped and passed through the [TFLite model](feature_extractor_int8.tflite) to generate its feature vector.  
- Cosine similarity is computed between the candidate feature vector and each reference feature vector.  
- If a candidate exceeds a set similarity threshold, it is flagged as a potential match and sent to the app for confirmation (app part to be implemented).

### **5. Non-Maximum Suppression (NMS)**  
- To eliminate overlapping detections, NMS merges duplicate bounding boxes, retaining only the most confident detection.

---

## **Testing the Object Recognition System**
To test the object detection pipeline normally (with the default demo reference/scene images and model path), run the following command:

```bash
python -m ranger_object_recognition.integration 
```

### **Testing with Custom Reference and/or scene images and model path**
To override the default reference, scene images and model path use the `--ref_dir`, `--scene` and `--model` arguments to provide a reference image directory, scene image and model path respectively. Example:

```bash
python -m ranger_object_recognition.integration \
    --ref_dir "ranger_object_recognition/local_test_stuff/wallet_refs" \
    --scene "ranger_object_recognition/local_test_stuff/wallet_scene1.jpeg" \
    --model "path/to/feature_extractor_int8.tflite"
```
