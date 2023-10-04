"""!
@package vision.scripts.block.Block
@file vision/scripts/block/Block.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-05-04

@brief Defines the abstract class Block.
"""
# ---------------------- IMPORT ----------------------
from abc import ABC, abstractmethod
from IPython.display import display
from PIL import Image

# ---------------------- GLOBAL CONSTANTS ----------------------

BLOCK_NAMES = [  'X1-Y1-Z2',
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

class Block:
    """
    @brief This class represents info of detected BLOCK
    """

    def __init__(self, name, conf, x1, y1, x2, y2, img_source_path):
        """
        Constructor

        :param name: name of block, ``str``
        :param conf: confidence of block, ``float``
        :param x1: x coordinate of top left corner, ``int``
        :param y1: y coordinate of top left corner, ``int``
        :param x2: x coordinate of bottom right corner, ``int``
        :param y2: y coordinate of bottom right corner, ``int``
        :param img_source_path: path to the image source, ``str``
        """

        self.name = name
        self.class_id = BLOCK_NAMES.index(name)
        self.confidence = conf
        self.xmin = x1
        self.ymin = y1
        self.xmax = x2
        self.ymax = y2
        self.img_source_path = img_source_path
        self.img_source = Image.open(self.img_source_path)
        self.center_point = (int((x1+x2)/2), int((y1+y2)/2))
        self.center_point_uv = (self.img_source.width - self.center_point[0], self.center_point[1])
        self.point_cloud = None
        self.transformation_matrix = None
        self.position = None
        self.rotation = None

    def get_name(self):
        """
        Get name of block
        :return (str): name of block, ``str``
        """
        return self.name

    def get_pixels(self):
        """
        Get pixels of block
        :return (list): list of pixels, ``list``
        """
        pixels = []
        for i in range(self.xmin, self.xmax):
            for j in range(self.ymin, self.ymax):
                pixels.append((i, j))
        return pixels

    def show(self):
        """
        Show detected block
        """

        self.img = self.img_source.crop((self.xmin, self.ymin, self.xmax, self.ymax))

        # Resize detected obj
        # Not neccessary. Useful when the obj is too small to see
        aspect_ratio = self.img.size[1] / self.img.size[0]
        new_width = 70  # resize width (pixel) for detected object to show
        new_size = (new_width, int(new_width * aspect_ratio))
        self.img = self.img.resize(new_size, Image.LANCZOS)

        # Block details
        display(self.img)
        
    def set_point_cloud(self, point_cloud):
        """
        Set point cloud of block
        :param point_cloud: point cloud of block, ``PointCloud``
        """
        self.point_cloud = point_cloud

    def set_transformation_matrix(self, transformation_matrix):
        """
        Set transformation matrix of block
        :param transformation_matrix: transformation matrix of block, ``np.ndarray``
        """
        self.transformation_matrix = transformation_matrix

    def set_pose(self, position, rotation):
        """
        Set pose of block
        :param position: position of block, ``np.ndarray``
        :param rotation: rotation of block, ``np.ndarray``
        """
        self.position = position
        self.rotation = rotation