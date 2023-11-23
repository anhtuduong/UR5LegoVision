import os
import sys
from pathlib import Path

# Resolve paths
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from PIL import Image
from constants import BLOCK_NAMES

def bounding_box_non_transparent(input_image_path, output_image_path):
    # Open the input image
    input_image = Image.open(input_image_path)

    # Get the alpha channel (transparency mask)
    alpha = input_image.split()[3]

    # Get the bounding box of the non-transparent region
    bbox = alpha.getbbox()

    # Create a new image with the bounding box size
    if bbox:
        output_image = input_image.crop(bbox)
        # Save the new image with the bounding box
        output_image.save(output_image_path)
    else:
        print("No non-transparent region found in the input image.")

for block_name in BLOCK_NAMES:
    input_image_path = os.path.abspath(os.path.join(ROOT, f"lego_builder/block_img/{block_name}.png"))
    output_image_path = os.path.abspath(os.path.join(ROOT, f"lego_builder/block_img/bbox_{block_name}.png"))
    bounding_box_non_transparent(input_image_path, output_image_path)

