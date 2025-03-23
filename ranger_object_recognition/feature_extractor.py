# feature_extractor.py

# Functions for loading the TFLite model and extracting feature vectors


import os
import tensorflow as tf
from tensorflow.lite.python.interpreter import Interpreter
import numpy as np
import glob
import cv2
import sys



def load_tflite_model(model_file):
    """Loads the quantized TFLite model and allocates tensors."""
    interpreter = Interpreter(model_path=model_file)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    return interpreter, input_details, output_details

def extract_feature_vector_tflite(interpreter, input_details, output_details, image_input):
    """
    Runs inference on image_input using the TFLite interpreter, and dequantises the output.
    Returns a flattened feature vector.
    """
    interpreter.set_tensor(input_details[0]['index'], image_input)
    interpreter.invoke()
    raw_features = interpreter.get_tensor(output_details[0]['index'])
    quant_params = output_details[0]['quantization']
    if quant_params is not None and quant_params[0] != 0:
        scale, zero_point = quant_params
        features = (raw_features.astype(np.float32) - zero_point) * scale
    else:
        features = raw_features.astype(np.float32)
    return features.flatten()

def load_feature_extractor(model_path):
    """
    Loads a feature extraction model as a TFSMLayer.
    Adjust 'call_endpoint' if needed (should prolly be 'serving_default').
    """
    feature_layer = tf.keras.layers.TFSMLayer(model_path, call_endpoint='serving_default')
    print(f"Loaded feature extractor from {model_path} as a TFSMLayer.", file=sys.stderr)
    return feature_layer

def extract_feature_vector(model, crop_input):
    """Runs the crop through the model and returns a flattened feature vector."""
    out = model(crop_input, training=False)
    # Print out the keys to debug (optional)
    print("Feature model output keys:", list(out.keys()), file=sys.stderr)
    key = list(out.keys())[0]  # take the first key available
    features = out[key]
    return features.numpy().flatten()

def load_reference_features(model, ref_dir, target_size=(224, 224)):
    """Extract feature vectors from each reference image in ref_dir."""
    ref_features = {}
    for ref_path in glob.glob(os.path.join(ref_dir, "*.*")):
        img = cv2.imread(ref_path)
        if img is None:
            continue
        # Convert BGR to RGB if necessary
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        crop_input = preprocess_crop(img, target_size)
        ref_vector = extract_feature_vector(model, crop_input)
        ref_features[ref_path] = ref_vector
    print(f"Extracted features from {len(ref_features)} reference images.", file=sys.stderr)
    return ref_features

def preprocess_crop(crop, target_size=(224, 224)):
    """Resize and normalize the crop from YOLO for the feature extractor."""
    if crop is None or crop.size == 0:
        print("Warning: encountered an empty crop. Skipping this crop.", file=sys.stderr)
        return None
    try:
        crop_resized = cv2.resize(crop, target_size)
    except Exception as e:
        print(f"Error resizing crop: {e}", file=sys.stderr)
        return None
    crop_normalized = crop_resized.astype(np.float32) / 255.0
    return np.expand_dims(crop_normalized, axis=0)