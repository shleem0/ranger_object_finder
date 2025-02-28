# feature_extractor.py

# Functions for loading the TFLite model and extracting feature vectors


import os
import tensorflow as tf
from tensorflow.lite.python.interpreter import Interpreter
import numpy as np

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