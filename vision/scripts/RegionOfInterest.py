"""!
@file RegionOfInterest.py
@author Giulio Zamberlan (giulio.zamberlan@studenti.unitn.it)
@brief Defines the class RegionOfInterest.py
@date 2023-02-17
"""
# ---------------------- IMPORT ----------------------
import cv2
import numpy as np
from pathlib import Path
import sys
import os

# ---------------------- GLOBAL CONSTANTS ----------------------
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
USING_REAL_CAM = False

# ---------------------- CLASS ----------------------

class RegionOfInterest:
    """
    @brief This class defines custom Region Of Interest
    """

    def __init__(self, image_path, output_path):
        """ @brief Class constructor
            @param img_path (String): path of input image
            @param output_path (String): path of out image
        """

        self.img_path = image_path
        self.output_path = output_path
        self.img = cv2.imread(self.img_path)

        # Let user choose ROI method
        choice = '0'
        while (choice != '1' and choice != ''):
            ask =  ('\nROI auto     (ENTER)'+
                    '\nROI manual   (1)'+
                    '\nchoice ----> ')
            choice = input(ask)

        # ROI auto
        if choice == '':
            print('ROI AUTO...')
            self.run_auto()

        # ROI manual
        if choice == '1':
            self.run_manual()

    def draw_box(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start = (x, y)
            self.end = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing == True:
                self.end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.end = (x, y)
            self.boxes.append((self.start, self.end))
            self.start = (-1, -1)
            self.end = (-1, -1)

    def run_manual(self):
        self.draw_img = self.img.copy()
        self.boxes = []
        self.drawing = False
        self.start = (-1, -1)
        self.end = (-1, -1)

        cv2.namedWindow('image')
        cv2.setMouseCallback('image', self.draw_box)
        while True:
            temp_img = self.draw_img.copy()
            for box in self.boxes:
                cv2.rectangle(temp_img, box[0], box[1], (0, 255, 0), 2)
            cv2.imshow('image', temp_img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == 13: # Check for "ENTER" button press
                mask = np.zeros(self.img.shape[:2], dtype=np.uint8)
                for box in self.boxes:
                    cv2.rectangle(mask, box[0], box[1], 255, -1)
                self.img[mask == 0] = (0, 0, 0)
                cv2.imwrite(self.output_path, self.img)
                self.draw_img[mask == 0] = (0, 0, 0)
                cv2.imshow('image', self.draw_img)
                cv2.waitKey(0)
                break

        cv2.destroyAllWindows()

    def run_auto(self):
        """ @brief automatically crop the region of interest depending on real camera or simulation camera
        """

        #create a mask of the same size of the image
        mask = np.zeros(self.img.shape[0:2], dtype=np.uint8)
        # check if the image is from real camera or simulation camera
        if USING_REAL_CAM:
            points = np.array([[[457,557], [555,272], [779,267], [960,532]]])
        else:
            points = np.array([[[845,409], [1201,412], [1545,913], [658, 921]]])

        # define the region of interest without aliasing
        cv2.drawContours(mask, [points], -1, (255, 255, 255), -1, cv2.LINE_AA)
    
        # apply the mask to the image
        res = cv2.bitwise_and(self.img,self.img,mask = mask)

        # save the image on the output path
        cv2.imwrite(self.output_path, res)

        # cv2.imshow("Samed Size White Image", dst)
        # cv2.waitKey(0)
        cv2.destroyAllWindows()

# ---------------------- MAIN ----------------------
# To use in command:
# python3 RegionOfInterest.py /path/to/input/img /path/to/output/img
if __name__ == '__main__':
    roi = RegionOfInterest(img_path=sys.argv[1], output_path=sys.argv[2])










