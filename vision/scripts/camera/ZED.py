
import rospy as ros
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

class ZED:

    def __init__(self):
        self.cv_image = None
        self.cv_bridge = CvBridge()
        self.image_sub = ros.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, self.receive_image)
        print('ZED initialized')

    def receive_image(self, data):
        """ @brief Callback function whenever take msg from ZED camera
            @param data (msg): data taken from ZED node
        """
        # Convert ROS image to OpenCV image
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
            print('image received')
        except CvBridgeError as e:
            print(e)


    def get_image(self):
        """ @brief Get image from ZED camera
            @return cv_image: image taken from ZED camera
        """
        return self.cv_image
    
    def save_image(self, path):
        """ @brief Save image from ZED camera
            @param path: path to save image
        """
        try:
            cv.imwrite(path, self.cv_image)
            print('image saved')
        except CvBridgeError as e:
            print(e)
        
