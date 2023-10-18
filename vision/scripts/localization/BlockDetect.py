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

# Import libraries
from IPython.display import Image, display
from PIL import Image as PILImage
from vision.scripts.block.Block import Block
from utils_ur5.Logger import Logger as log
from constants import LOG_FOLDER, IMG_RESULT_PATH

# Global constants
DETECT_PATH = os.path.abspath("./runs/detect/predict")
RUN_PATH = os.path.abspath("./runs")
CONFIDENCE = 0.25

# YOLOv5
# WEIGHTS_PATH = os.path.abspath(os.path.join(ROOT, "vision/weights/best.pt"))
# MODEL = torch.hub.load('ultralytics/yolov5', 'custom', WEIGHTS_PATH, force_reload=True)

# YOLOv8
from ultralytics import YOLO
WEIGHTS_PATH = os.path.abspath(os.path.join(ROOT, "vision/yolov8-training/weights/best.pt"))
model = YOLO(WEIGHTS_PATH)

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

    def __init__(self, img_path, save_result=False, open_result=False):
        """
        Constructor
        :param img_path: path to the image source, ``str``
        """
        log.debug_highlight('DETECTING BLOCKS...')
    
        self.block_list = []
        self.detect(img_path, open_result)

        if save_result:
            self.move_detect_result_to(LOG_FOLDER)

        # Clean
        self.clean_runs_folder()


    def detect(self, img_path, open_result=False):
        """
        This function pass the image path to the model and calculate bounding boxes for each object
        :param img_path: path to the image source, ``str``
        """
        self.block_list.clear()

        # Detection model
        log.debug_highlight(f'Using YOLOv8: {WEIGHTS_PATH}')
        self.results = model.predict(source=img_path, conf=CONFIDENCE, save=True, line_width=2)
        result = self.results[0]

        # Bounding boxes
        boxes = result.boxes
        for box in boxes:
            id = int(box.cls)
            name = result.names[id]
            conf = round(float(box.conf), 2)
            x1 = int(box.xyxy[0][0])
            y1 = int(box.xyxy[0][1])
            x2 = int(box.xyxy[0][2])
            y2 = int(box.xyxy[0][3])
            # Add block to list
            self.block_list.append(Block(name, conf, x1, y1, x2, y2, img_path))

        # Info
        log.info(f'Detected {len(self.block_list)} object(s)')
        self.show_detected_block(open_result)

    def show_detected_block(self, open_result=False):
        """
        This function show infos of detected blocks
        """
        for index, block in enumerate(self.block_list, start=1):
            log.debug(f'{index}: {block.name} conf({block.confidence}) xyxy({block.xmin}, {block.ymin}, {block.xmax}, {block.ymax})')
            block.show()

        # Open result image
        for file in os.listdir(DETECT_PATH):
            img_result = os.path.join(DETECT_PATH, file)
            display(Image(filename=img_result))
        if open_result:
            im = PILImage.open(img_result)
            im.show()
    
    def move_detect_result_to(self, path):
        """
        This function move the detect image to the path
        :param path: path to the destination, ``str``
        """
        for file in os.listdir(DETECT_PATH):
            # Append file name with _result
            file_name = file.split('.')[0] + '_result.png'
            file_path = os.path.join(DETECT_PATH, file)
            file_path_result = os.path.join(path, file_name)
            # Move file to path
            os.replace(file_path, file_path_result)
            log.warning(f'Moved {file_name} to {path}')

    def clean_runs_folder(self):
        """
        This function clean and remove the runs folder
        """
        # Remove all files in detect folder
        for file in os.listdir(DETECT_PATH):
            os.remove(os.path.join(DETECT_PATH, file))
        os.rmdir(DETECT_PATH)
        os.rmdir(os.path.join(RUN_PATH, 'detect'))
        os.rmdir(RUN_PATH)


# ---------------------- MAIN ----------------------

if __name__ == '__main__':
    img_test = os.path.abspath(os.path.join(ROOT, "logs/img_ZED_cam.png"))
    detect = BlockDetect(img_test)

