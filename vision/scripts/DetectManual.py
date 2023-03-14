"""!
@file DetectManual.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief Defines the class DetectManual.py
@date 2023-02-19
"""
# ---------------------- IMPORT ----------------------
import cv2
import numpy as np
from pathlib import Path
import sys
import os
from Lego import Lego

# ---------------------- GLOBAL CONSTANTS ----------------------
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

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

class DetectManual:
    """
    @brief
    """

    def __init__(self, image_path):
        """ @brief Class constructor
            @param img_path (String): path of input image
            @param output_path (String): path of out image
        """

        self.image_path = image_path
        self.class_list = LEGO_NAMES
        self.point_list = []
        self.lego_list = []

        self.select_points()

    def select_points(self):
        self.image = cv2.imread(self.image_path)
        cv2.namedWindow("image")
        cv2.imshow("image", self.image)
        cv2.setMouseCallback("image", self._on_mouse)

        while True:
            key = cv2.waitKey(1)
            if key == 13:  # ENTER key
                break

        # cv2.waitKey(0)

        cv2.destroyAllWindows()

        for point in self.point_list:
            name = point[2]
            center_point = (point[0], point[1])
            x1 = point[0] - 20
            y1 = point[1] - 20
            x2 = point[0] + 20
            y2 = point[1] + 20
            lego = Lego(name, 100, x1, y1, x2, y2,self.image_path)
            lego.center_point = center_point
            self.lego_list.append(lego)
            

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            class_window = "Choose a class"
            for i, class_name in enumerate(self.class_list):
                cv2.namedWindow(class_name)
                cv2.moveWindow(class_name, 10, 10 + 50 * i)
                cv2.imshow(class_name, np.zeros((50, 100), dtype=np.uint8))
                cv2.setMouseCallback(class_name, self._on_button_click, (x, y, class_name))
            cv2.rectangle(self.image, (x-5, y-5), (x+5, y+5), (0, 255, 0), -1)
            cv2.imshow("image", self.image)

        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.point_list:
                self.point_list.pop()

    def _on_button_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            class_name = param[2]
            original_x, original_y = param[0], param[1]
            self.point_list.append((original_x, original_y, class_name))
            for class_name in self.class_list:
                cv2.destroyWindow(class_name)

# ---------------------- MAIN ----------------------

if __name__ == '__main__':
    detectManual = DetectManual(img_path=sys.argv[1])










