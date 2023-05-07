
import rospy as ros
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

class ZED:

    def __init__(self):
        self.image_sub = ros.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, self.receive_image)
        
    def receive_image(self, msg):
        """ @brief Callback function whenever take msg from ZED camera
            @param msg: data taken from ZED node
        """
        try:
            self.img_received = msg
        except Exception as e:
            print(e)

    def get_image(self):
        """ @brief Get image from ZED camera
            @return cv_image: image taken from ZED camera
        """
        # Convert ROS image to OpenCV image
        try:
            cv_image = CvBridge().imgmsg_to_cv2(self.img_received, "bgr8")
        except CvBridgeError as e:
            print(e)

        return cv_image
    
    def save_image(self, path, img):
        """ @brief Save image from ZED camera
            @param path: path to save image
        """
        try:
            cv.imwrite(path, img)
        except CvBridgeError as e:
            print(e)
        
