"""!
@file Vision.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it), Giulio Zamberlan (giulio.zamberlan@studenti.unitn.it)
@brief Defines the Vision node that communicates with Motion node.
@date 2023-02-17
"""

# ---------------------- IMPORT ----------------------
from pathlib import Path
import sys
import os
import rospy as ros
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32
from motion.msg import pos
from LegoDetect import LegoDetect

# ---------------------- GLOBAL CONSTANTS ----------------------
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
IMG_ZED = os.path.abspath(os.path.join(ROOT, "log/img_ZED_cam.png"))

w_R_c = np.matrix([[0, -0.499, 0.866], [-1, 0, 0], [0, -0.866, -0.499]])
x_c = np.array([-0.9, 0.24, -0.35])
base_offset = np.array([0.5, 0.35, 1.75])

OFF_SET = 0.86 + 0.1
REAL_ROBOT = False

# ---------------------- CLASS ----------------------

class Vision:
    """
    @brief This class recognizes lego blocks from ZED camera and communicates with different ROS node
    """

    def __init__(self):
        """ @brief Class constructor
        """

        ros.init_node('vision', anonymous=True)

        self.lego_list = []
        self.bridge = CvBridge()

        # Flags
        self.allow_receive_image = True
        self.allow_receive_pointcloud = False
        self.vision_ready = 0

        # Subscribe and publish to ros nodes
        self.image_sub = ros.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, self.receive_image)
        self.pointcloud_sub = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, self.receive_pointcloud, queue_size=1)
        self.pos_pub = ros.Publisher("/vision/pos", pos, queue_size=1)
        self.ack_sub = ros.Subscriber('/vision/ack', Int32, self.ackCallbak)
        self.ack_pub = ros.Publisher('/taskManager/stop', Int32, queue_size=1)
        
    def receive_image(self, data):
        """ @brief Callback function whenever take msg from ZED camera
            @param data (msg): msg taken from ZED node
        """

        # Flag
        if not self.allow_receive_image:
            return
        self.allow_receive_image = False

        # Convert ROS image to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Save image and detect lego
        cv.imwrite(IMG_ZED, cv_image)
        legoDetect = LegoDetect(IMG_ZED)
        self.lego_list = legoDetect.lego_list

        self.allow_receive_pointcloud = True

    def receive_pointcloud(self, msg):
        """ @brief Callback function whenever take point_cloud msg from ZED camera
            @param msg (msg): msg taken from ZED node
        """

        # Flag
        if not self.allow_receive_pointcloud:
            return
        self.allow_receive_pointcloud = False
        
        self.pos_msg_list = []

        for lego in self.lego_list:

            # Get point cloud
            for data in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=True, uvs=[lego.center_point]):
                lego.point_cloud = (data[0], data[1], data[2])

            if REAL_ROBOT:
                lego.point_world = lego.point_cloud
            else:
                # Transform point cloud to world
                lego.point_world = w_R_c.dot(lego.point_cloud) + x_c + base_offset

            # Show details
            lego.show()

            # Create msg for pos_pub
            pos_msg = pos()
            pos_msg.class_id = lego.class_id
            pos_msg.x = lego.point_world[0, 0] + 0.007
            pos_msg.y = lego.point_world[0, 1]
            pos_msg.z = lego.point_world[[0, 2]]
            pos_msg.pitch = 0
            pos_msg.roll = 0
            pos_msg.yaw = 0

            if pos_msg.z < OFF_SET:
                self.pos_msg_list.append(pos_msg)
            
        print('\nVISION DONE DETECTING LEGO!\nREADY FOR MOTION!')
        self.vision_ready = 1
        self.send_pos_msg()

    def ackCallbak(self, ack_ready):
        """ @brief check if the motion planner is ready to receive the position of the lego
            @param ack_ready (msg): msg from Motion node
        """

        if self.vision_ready == 1 and ack_ready.data == 1:
            self.send_pos_msg()
            
    def send_pos_msg(self):
        """ @brief send the position of the lego to motion planner
        """
        try:
            pos_msg = self.pos_msg_list.pop()
            self.pos_pub.publish(pos_msg)
            print('\nPosition published:\n', pos_msg)
        except IndexError:
            print('\nFINISH ALL LEGO\n')
            
# ---------------------- MAIN ----------------------
# To use in command:
# python3 Vision.py
 
if __name__ == '__main__':

    vision = Vision()

    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
