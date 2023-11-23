"""!
@file LegoConstructor.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-11-20

@brief Defines the class LegoConstructor.
This class is used to identify and place lego blocks on a given image.
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

import cv2
import numpy as np
from utils_ur5.Logger import Logger as log

THRESHOLD = 0.1

# from constants import BLOCK_NAMES
BLOCK_NAMES = [ 
    # 'X1-Y1-Z2',
    # 'X1-Y2-Z1',
    # 'X1-Y2-Z2',
    'X1-Y2-Z2-CHAMFER',
    # 'X1-Y2-Z2-TWINFILLET',
    # 'X1-Y3-Z2',
    # 'X1-Y3-Z2-FILLET',
    # 'X1-Y4-Z1',
    # 'X1-Y4-Z2',
    # 'X2-Y2-Z2',
    # 'X2-Y2-Z2-FILLET'
]

INPUT_IMG_PATH = '/home/toto/ros_ws/src/UR5LegoVision/lego_builder/output/output.png'

class LegoConstructor:
    def __init__(self, input_img_path):
        self.input_img = cv2.imread(input_img_path)
        self.input_img_gray = cv2.cvtColor(self.input_img, cv2.COLOR_BGR2GRAY)
        self.matches = {}

        for block_name in BLOCK_NAMES:
            block_img_path = os.path.abspath(os.path.join(ROOT, f"lego_builder/block_img/{block_name}.png"))
            self.add_template(block_name, block_img_path)

    def add_template(self, block_name, block_image_path):
        template_img = self.preprocess_image(block_image_path)
        cv2.imshow('Preprocessed', template_img)
        # template_img = self.extract_shape(template_img)
        # cv2.imshow('Shape', template_img)

        similarity, match_location = self.template_matching(template_img)
        self.matches[block_name] = {'Similarity': similarity, 'Location': match_location, 'Template': template_img}

    def preprocess_image(self, img_path, resize_px=400):
        template_img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        
        # Resize image
        template_img = cv2.resize(template_img, (resize_px, resize_px))
        # cv2.imshow(f'Template resized {resize_px}x{resize_px}', template_img)

        # Fill the transparent parts with white
        # if template_img.shape[2] == 4:
        #     alpha_channel = template_img[:, :, 3]
        #     _, mask = cv2.threshold(alpha_channel, 254, 255, cv2.THRESH_BINARY)
        #     color = template_img[:, :, :3]
        #     template_img = cv2.bitwise_not(cv2.bitwise_not(color, mask=mask))
            
        # Convert to grayscale
        template_img = cv2.cvtColor(template_img, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Template Gray', template_img)
        
        return template_img

    def extract_shape(self, img):
        _, binary_mask = cv2.threshold(img, 1, 255, cv2.THRESH_BINARY)  # Create binary mask
        return binary_mask

    def template_matching(self, template):
        result = cv2.matchTemplate(self.input_img_gray, template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        return max_val, max_loc
    
    def identify_blocks(self, threshold=0.8):
        for block_name, data in self.matches.items():
            similarity = data['Similarity']
            match_location = data['Location']
            template_img = data['Template']
            log.debug(f"{block_name}: Similarity {similarity}, Location {match_location}")

            if similarity > threshold:
                log.info(f"{block_name}: Best match found at {match_location} with similarity {similarity}")
                # Draw rectangle around the matched region
                h, w = template_img.shape[::-1]
                top_left = match_location
                bottom_right = (top_left[0] + w, top_left[1] + h)
                cv2.rectangle(self.input_img, top_left, bottom_right, (0, 255, 0), 2)

                # Paste the block image onto the input image
                # self.input_img = self.paste_image(self.input_img, template_img, match_location)
                cv2.imshow('Matching', template_img)

        # Display the result
        cv2.imshow('Matched Blocks', self.input_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def paste_image(self, background_img, overlay_img, position=(0, 0)):
        # Get the dimensions of the overlay image
        overlay_height, overlay_width = overlay_img.shape[:2]

        # Define the region of interest (ROI) for pasting the overlay image onto the background image
        y_end, x_end = position[1] + overlay_height, position[0] + overlay_width

        # Ensure the overlay image has an alpha channel (if using transparency)
        if overlay_img.shape[2] == 3:
            overlay_img = cv2.cvtColor(overlay_img, cv2.COLOR_BGR2BGRA)

        # Blend the images using addWeighted function
        added_image = cv2.addWeighted(background_img[position[1]:y_end, position[0]:x_end],
                                    1.0,
                                    overlay_img[:overlay_height, :overlay_width],
                                    1.0,
                                    0)

        # Paste the blended part onto the background image
        background_img[position[1]:y_end, position[0]:x_end] = added_image

        return background_img

if __name__ == "__main__":
    castle_builder = LegoConstructor(INPUT_IMG_PATH)
    castle_builder.identify_blocks(threshold=THRESHOLD)  # Adjust threshold as needed