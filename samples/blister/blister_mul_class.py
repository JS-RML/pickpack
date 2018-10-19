"""
Mask R-CNN
Copyright (c) 2018 Matterport, Inc.
Licensed under the MIT License (see LICENSE for details)
Written by Waleed Abdulla

Used by Z. Tong (The Hong Kong University of Science and Technology)
for training a Pharmaceutical Blister Packs dataset
and implement instance segmentation.
"""

from mrcnn.config import Config

############################################################
#  Configurations
############################################################

class BlisterConfig(Config):
    """Configuration for training on the toy  dataset.
    Derives from the base Config class and overrides some values.
    """
    # Give the configuration a recognizable name
    NAME = "blister"

    # We use a GPU with 12GB memory, which can fit two images.
    # Adjust down if you use a smaller GPU.
    IMAGES_PER_GPU = 1

    # Number of classes (including background)
    NUM_CLASSES = 1 + 2  # Background + blister

    # Number of training steps per epoch
    STEPS_PER_EPOCH = 362 

    # Skip detections with < 90% confidence
    DETECTION_MIN_CONFIDENCE = 0.9



