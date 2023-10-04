"""!
@file BlockDetect.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-05-04

@brief Defines the class BlockDetect.
"""

import os
import sys
from pathlib import Path

# Resolve paths
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import torch
from matplotlib import pyplot as plt
import numpy as np
import cv2 as cv
from IPython.display import display
from PIL import Image
from vision.scripts.localization.RegionOfInterest import RegionOfInterest
from DetectManual import DetectManual
from vision.scripts.block.Block import Block

# Global constants
IMG_ROI_PATH = os.path.abspath(os.path.join(ROOT, "logs/img_ROI.png"))
WEIGHTS_PATH = os.path.abspath(os.path.join(ROOT, "vision/weights/best.pt"))
CONFIDENCE = 0.7
MODEL = torch.hub.load('ultralytics/yolov5', 'custom', WEIGHTS_PATH, force_reload=True)

BLOCK_NAMES = [ 'X1-Y1-Z2',
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

class BlockDetect:
    """
    This class use custom trained weights and detect blocks with YOLOv5
    """

    def __init__(self, img_path):
        """
        Constructor
        :param img_path: path to the image source, ``str``
        """

        MODEL.conf = CONFIDENCE
        MODEL.multi_label = False
        # MODEL.iou = 0.5
    
        self.block_list = []
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
        """
        Detect manually
        :param img_path: path to the image source, ``str``
        """
        self.block_list.clear()
        detectManual = DetectManual(img_path)
        self.block_list = detectManual.block_list
        
        # Info
        print('Detected', len(self.block_list), 'object(s)\n')
        self.show()

    def detect_ROI(self, img_path):
        """
        Detect using Region Of Interest
        :param img_path: path to the image source, ``str``
        """

        print('Draw RegionOfInterest')
        roi = RegionOfInterest(img_path, IMG_ROI)
        print('Detecting RegionOfInterest...')
        self.detect(IMG_ROI)

    def detect(self, img_path):
        """
        This function pass the image path to the model and calculate bounding boxes for each object
        :param img_path: path to the image source, ``str``
        """
        self.block_list.clear()

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
            # Add block to list
            self.block_list.append(Block(name, conf, x1, y1, x2, y2, img_path))

        # Info
        print('Detected', len(self.block_list), 'object(s)\n')
        self.show()

    def show(self):
        """
        This function show infos of detected blocks
        """
        for index, block in enumerate(self.block_list, start=1):
            print(index)
            block.show()

    def get_block_list(self): 
        """
        This function return the list of detected blocks
        :return (list): list of detected blocks, ``list``
        """
        return self.block_list


# ---------------------- MAIN ----------------------
# To use in command:
# python3 BlockDetect.py /path/to/img...

if __name__ == '__main__':
    BlockDetect = BlockDetect(img_origin_path=sys.argv[1])

