"""!
@file Lego.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief Defines the class Lego.
@date 2023-02-17
"""
# ---------------------- IMPORT ----------------------
from IPython.display import display
from PIL import Image

# ---------------------- GLOBAL CONSTANTS ----------------------

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

class Lego:
    """
    @brief This class represents info of detected lego
    """

    def __init__(self, name, conf, x1, y1, x2, y2, img_source_path):
        """ @brief Class constructor
            @param name (String): lego name
            @param conf (float): confidence
            @param x1 (float): xmin of bounding box
            @param y1 (float): ymin of bounding box
            @param x2 (float): xmax of bounding box
            @param y2 (float): ymax of bounding box
            @param img_source_path (String): path to image
        """

        self.name = name
        self.class_id = LEGO_NAMES.index(name)
        self.confidence = conf
        self.xmin = x1
        self.ymin = y1
        self.xmax = x2
        self.ymax = y2
        self.img_source_path = img_source_path
        self.img_source = Image.open(self.img_source_path)
        self.center_point = (int((x1+x2)/2), int((y1+y2)/2))
        self.center_point_uv = (self.img_source.width - self.center_point[0], self.center_point[1])
        self.point_cloud = ()
        self.point_world = ()

    def show(self):
        """ @brief Show lego info
        """

        self.img = self.img_source.crop((self.xmin, self.ymin, self.xmax, self.ymax))

        # Resize detected obj
        # Not neccessary. Useful when the obj is too small to see
        aspect_ratio = self.img.size[1] / self.img.size[0]
        new_width = 70  # resize width (pixel) for detected object to show
        new_size = (new_width, int(new_width * aspect_ratio))
        self.img = self.img.resize(new_size, Image.LANCZOS)

        # Lego details
        display(self.img)
        print('class =', self.name)
        print('id =', self.class_id)
        print('confidence =', '%.2f' %self.confidence)
        print('center_point =', self.center_point)
        print('center_point_uv =', self.center_point_uv)
        print('--> point cloud =', self.point_cloud)
        print('--> point world =', self.point_world)
        print()