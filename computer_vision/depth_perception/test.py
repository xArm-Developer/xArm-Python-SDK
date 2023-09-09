from detectron2.engine import DefaultPredictor

import os
import pickle

from utils import *


######### LOADING PREDICTOR ##########
# gets the saved cfg
# saved in the pickle file in train.py
cfg_save_path = "OD_cfg.pickle"
with open(cfg_save_path, 'rb') as f: # rb means read binary
    cfg = pickle.load(f)

# input trained model
cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, "model_final.pth")
# confidence interval threshold
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.3

predictor = DefaultPredictor(cfg)


######### TESTING #########
image_path = "pipet.jpg"
#video_path = "test/test-tube.avi" # O for live video

on_image(image_path, predictor)
#on_video(video_path, predictor)
