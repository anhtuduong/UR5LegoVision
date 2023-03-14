"""!
@file LegoDetect.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief Defines the class LegoDetect.
@date 2023-02-17
"""
# ---------------------- IMPORT ----------------------
from pathlib import Path
import sys
import os
import torch
from matplotlib import pyplot as plt
import numpy as np
import cv2 as cv
from IPython.display import display
from PIL import Image
from RegionOfInterest import RegionOfInterest
from DetectManual import DetectManual
from Lego import Lego

# ---------------------- GLOBAL CONSTANTS ----------------------
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
VISION_PATH = os.path.abspath(os.path.join(ROOT, ".."))
IMG_ROI = os.path.abspath(os.path.join(ROOT, "log/img_ROI.png"))

WEIGHTS_PATH = os.path.join(VISION_PATH, "weights/best.pt")
CONFIDENCE = 0.7
MODEL = torch.hub.load('ultralytics/yolov5', 'custom', WEIGHTS_PATH)

LEGO_NAMES = [  'X1-Y1-Z2',
                'X1-Y2-Z1',
                'X1-Y2-Z2',
                'X1-Y2-Z2-CHAMFER',
                'X1-Y2-Z2-TWINFILLET',
                'X1-Y3-Z2',
                'X1-Y3-Z2-FILLET',
                'X1-Y4-Z1',
                'X1-Y4-Z2',
                'X2-Y2-Z2',
                'X2-Y2-Z2-FILLET']

# ---------------------- CLASS ----------------------

class LegoDetect:
    """
    @brief This class use custom trained weights and detect lego blocks with YOLOv5
    """

    def __init__(self, img_path):
        """ @brief Class constructor
            @param img_path (String): path of input image
        """

        MODEL.conf = CONFIDENCE
        MODEL.multi_label = False
        # MODEL.iou = 0.5
    
        self.lego_list = []
        self.detect(img_path)

        # Let user choose detect method
        choice = '0'
        while True:
            while (choice != '1' and choice != '2' and choice != '3' and choice != ''):
                ask =  ('\nContinue         (ENTER)'+
                        '\nDetect again     (1)'+
                        '\nDetect ROI       (2)'+
                        '\nDetect manually  (3)'+
                        '\nchoice ----> ')
                choice = input(ask)

            # Continue
            if choice == '':
                break

            # Detect again using original image
            if choice == '1':
                print('Detecting again...')
                self.detect(img_path)
                choice = '0'

            # Detect using ROI
            if choice == '2':
                self.detect_ROI(img_path)
                choice = '0'

            # Detect manually
            if choice == '3':
                print('Detect manually')
                self.detect_manual(img_path)
                choice = '0'

    def detect_manual(self, img_path):
        self.lego_list.clear()
        detectManual = DetectManual(img_path)
        self.lego_list = detectManual.lego_list
        
        # Info
        print('Detected', len(self.lego_list), 'object(s)\n')
        self.show()

    def detect_ROI(self, img_path):
        """ @brief Detect using Region Of Interest
            @param img_path (String): path of input image
        """

        print('Draw RegionOfInterest')
        roi = RegionOfInterest(img_path, IMG_ROI)
        print('Detecting RegionOfInterest...')
        self.detect(IMG_ROI)

    def detect(self, img_path):
        """ @brief This function pass the image path to the model and calculate bounding boxes for each object
            @param img_path (String): path of input image
        """
        self.lego_list.clear()

        # Detection model
        self.results = MODEL(img_path)
        self.results.show()
        img = Image.open(img_path)
        print(img_path)
        print('img size:', img.width, 'x', img.height)

        # Bounding boxes
        bboxes = self.results.pandas().xyxy[0].to_dict(orient="records")
        for bbox in bboxes:
            name = bbox['name']
            conf = bbox['confidence']
            x1 = int(bbox['xmin'])
            y1 = int(bbox['ymin'])
            x2 = int(bbox['xmax'])
            y2 = int(bbox['ymax'])
            # Add lego to list
            self.lego_list.append(Lego(name, conf, x1, y1, x2, y2, img_path))

        # Info
        print('Detected', len(self.lego_list), 'object(s)\n')
        self.show()

    def show(self):
        """ @brief This function show infos of detected legos
        """
        for index, lego in enumerate(self.lego_list, start=1):
            print(index)
            lego.show()


# ---------------------- MAIN ----------------------
# To use in command:
# python3 LegoDetect.py /path/to/img...

if __name__ == '__main__':
    legoDetect = LegoDetect(img_origin_path=sys.argv[1])

