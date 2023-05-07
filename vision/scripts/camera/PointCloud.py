
import rospy as ros
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2

class PointCloud:

    def __init__(self):
        self.pointcloud_sub = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, self.receive_pointcloud, queue_size=1)

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

        return point_cloud



if __name__ == '__main__':
    pc = PointCloud()
    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
