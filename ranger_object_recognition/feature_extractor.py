import os
import tensorflow as tf

def load_feature_extractor(model_path):
    """
    Loads the SavedModel feature extractor from the given (absolute) path.
    """
    abs_path = os.path.abspath(model_path)
    model = tf.saved_model.load(abs_path)
    print("Model loaded successfully from:", abs_path)
    return model

def extract_feature_vector(model, image_input):
    """
    Runs the model on the preprocessed image and returns a flattened feature vector.
    """
    features = model(image_input)
    return features.numpy().flatten()