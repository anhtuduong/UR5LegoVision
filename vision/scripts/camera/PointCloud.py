
# Reslve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# Import
import rospy as ros
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2
from vision.scripts.utils.Logger import Logger as log

class PointCloud:

    def __init__(self):
        self.pointcloud_received = None
        self.pointcloud_sub = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, self.receive_pointcloud, queue_size=1)
        # wait for point cloud to be ready
        ros.sleep(2)
        log.debug('Point cloud initialized')

    def receive_pointcloud(self, msg):
        """ @brief Callback function whenever take point_cloud msg from ZED camera
            @param msg (msg): msg taken from ZED node
        """
        self.pointcloud_received = msg

    def get_pointcloud(self, pixels):
        """ @brief Get point_cloud from ZED camera
            @param pixels (list): list of pixels
            @return point_cloud (list): list of point_cloud
        """
        point_cloud = []
        for pixel in pixels:
            for data in point_cloud2.read_points(self.pointcloud_received, field_names=['x','y','z'], skip_nans=True, uvs=[pixel]):
                point_cloud.append((data[0], data[1], data[2]))
                point_cloud2.

        log.debug('point_cloud returned')
        return point_cloud


if __name__ == '__main__':
    pc = PointCloud()
    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
