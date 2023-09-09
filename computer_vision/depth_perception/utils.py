# helper functions for train and test.py

from detectron2.data import DatasetCatalog, MetadataCatalog
from detectron2.utils.visualizer import Visualizer
from detectron2.config import get_cfg
from detectron2 import model_zoo

from detectron2.utils.visualizer import ColorMode

import random
import cv2
import matplotlib.pyplot as plt

# plots the boundary box of the detected objects from datasets we registered
# used to confirm that samples and annotations are working
# dataset_name is the name we set when we registered the coco instances in train.py
# n is number of images to randomly select
def plot_samples(dataset_name, n=1):
    # gets the dataset information and metadata
    dataset_custom = DatasetCatalog.get(dataset_name)
    dataset_custom_metadata = MetadataCatalog.get(dataset_name)

    # randomly selects instances from the dataset we inputted
    for s in random.sample(dataset_custom, n):
        # read the image
        img = cv2.imread(s["file_name"])
        
        # !!
        v = Visualizer(img[:,:,::-1], metadata=dataset_custom_metadata, scale=0.5)
        v = v.draw_dataset_dict(s)
        plt.figure(figsize=(15,20))
        plt.imshow(v.get_image())
        plt.show()


# sets the configuration defaults
def get_train_cfg(config_file_path, checkpoint_url, train_dataset_name, test_dataset_name, num_classes, device, output_dir):
    cfg = get_cfg()

    cfg.merge_from_file(model_zoo.get_config_file(config_file_path)) # !!
    cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(checkpoint_url) # !!
    cfg.DATASETS.TRAIN = (train_dataset_name,) # !! why comma
    cfg.DATASETS.TEST = (test_dataset_name,)

    # more workers is faster but stresses CPU more
    cfg.DATALOADER.NUM_WORKERS = 2

    cfg.SOLVER.IMS_PER_BATCH = 2
    # learning rate
    cfg.SOLVER.BASE_LR = 0.00025
    # attritions
    cfg.SOLVER.MAX_ITER = 1000
    cfg.SOLVER.STEPS = []

    cfg.MODEL.ROI_HEADS.NUM_CLASSES = num_classes
    cfg.MODEL.DEVICE = device # !!
    # sets the output dir where model is saved
    cfg.OUTPUT_DIR = output_dir # !!

    return cfg


# run the model on a selected image
def on_image(image_path, predictor):
    img = cv2.imread(image_path)
    outputs = predictor(img)
    
    # !!
    v = Visualizer(img[:,:,::-1], metadata = {}, scale = 0.5, instance_mode = ColorMode.SEGMENTATION)
    v = v.draw_instance_predictions(outputs["instances"].to("cpu"))
    plt.figure(figsize=(14,10))
    plt.imshow(v.get_image())
    plt.show()


# run model on a selected video path
def on_video(video_path, predictor):
    cap = cv2.VideoCapture(video_path)
    if (cap.isOpened() == False):
        print("Error opening file")
        return

    success, image = cap.read()
    while success:
        predictions = predictor(image)
        v = Visualizer(image[:,:,::-1], metadata={}, instance_mode=ColorMode.SEGMENTATION)
        output = v.draw_instance_predictions(predictions["instances"].to("cpu"))

        cv2.imshow("Result", output.get_image()[:,:,::-1])

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        success, image = cap.read()