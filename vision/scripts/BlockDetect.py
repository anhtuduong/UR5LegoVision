"""!
@file BlockDetect.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief Defines the class BlockDetect.
@date 2023-05-04
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
<<<<<<< HEAD:vision/scripts/localization/BlockDetect.py
from vision.scripts.block.Block import Block
=======
import Block
>>>>>>> change name:vision/scripts/BlockDetect.py

# Global constants
IMG_ROI_PATH = os.path.abspath(os.path.join(ROOT, "logs/img_ROI.png"))
WEIGHTS_PATH = os.path.abspath(os.path.join(ROOT, "vision/weights/best.pt"))
CONFIDENCE = 0.7
MODEL = torch.hub.load('ultralytics/yolov5', 'custom', WEIGHTS_PATH, force_reload=True)

<<<<<<< HEAD:vision/scripts/localization/BlockDetect.py
BLOCK_NAMES = [ 'X1-Y1-Z2',
=======
block_NAMES = [  'X1-Y1-Z2',
>>>>>>> change name:vision/scripts/BlockDetect.py
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
    @brief This class use custom trained weights and detect blocks with YOLOv5
    """

    def __init__(self, img_path):
        """ @brief Class constructor
            @param img_path (String): path of input image
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
        self.block_list.clear()
        detectManual = DetectManual(img_path)
        self.block_list = detectManual.block_list
        
        # Info
        print('Detected', len(self.block_list), 'object(s)\n')
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
        """ @brief This function show infos of detected blocks
        """
        for index, block in enumerate(self.block_list, start=1):
            print(index)
            block.show()
<<<<<<< HEAD:vision/scripts/localization/BlockDetect.py

    def get_block_list(self): 
        """ @brief This function return the list of detected blocks
            @return block_list (list): list of detected blocks
        """
        return self.block_list
=======
>>>>>>> change name:vision/scripts/BlockDetect.py


# ---------------------- MAIN ----------------------
# To use in command:
# python3 BlockDetect.py /path/to/img...

if __name__ == '__main__':
    BlockDetect = BlockDetect(img_origin_path=sys.argv[1])

