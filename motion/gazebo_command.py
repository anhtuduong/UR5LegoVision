"""!
@package motion.gazebo_command
@file motion/gazebo_command.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-05-25

@brief Defines the GazeboCommand class that sends commands to Gazebo
"""

# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

# Import
import locosim.robot_control.base_controllers.params as conf
import json
from utils_ur5.Logger import Logger as log
from constants import MODEL, LEGO_SCENE
from utils_ur5.xml_utils import *

# ROS
import rospy
from gazebo_msgs.srv import SetModelState, GetModelState, GetModelStateRequest
from gazebo_msgs.srv import SetLinkState, GetLinkState, GetLinkStateRequest
from gazebo_msgs.msg import ModelState, LinkState
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

# ---------------------- CLASS ----------------------

class GazeboCommand():
    """
    The class that sends commands to Gazebo
    """

    def __init__(self):
        """
        Constructor
        """
        self.get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_link_state_proxy = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.set_link_state_proxy = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        self.spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.attach_proxy = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_proxy = rospy.ServiceProxy('/link_attacher_node/detach', Attach)

    def attach_models(self, model_name_1, link_name_1, model_name_2, link_name_2):
        """
        Command that attaches model_1 to model_2
        :param model_name_1: name of the first model, ``str``
        :param link_name_1: name of the first link, ``str``
        :param model_name_2: name of the second model, ``str``
        :param link_name_2: name of the second link, ``str``
        :exception rospy.ServiceException: if the service call failed
        """
        rospy.wait_for_service('/link_attacher_node/attach')
        try:
            req = AttachRequest()
            req.model_name_1 = model_name_1
            req.link_name_1 = link_name_1
            req.model_name_2 = model_name_2
            req.link_name_2 = link_name_2
            self.attach_proxy(req)
            log.warning(f'Attached {model_name_1} to {model_name_2}')
        except rospy.ServiceException as e:
            log.error(f'Failed to attach {model_name_1} to {model_name_2}: {str(e)}')

    def dettach_models(self, model_name_1, link_name_1, model_name_2, link_name_2):
        """
        Command that detaches model_1 from model_2
        :param model_name_1: name of the first model, ``str``
        :param link_name_1: name of the first link, ``str``
        :param model_name_2: name of the second model, ``str``
        :param link_name_2: name of the second link, ``str``
        :exception rospy.ServiceException: if the service call failed
        """
        rospy.wait_for_service('/link_attacher_node/detach')
        try:
            req = AttachRequest()
            req.model_name_1 = model_name_1
            req.link_name_1 = link_name_1
            req.model_name_2 = model_name_2
            req.link_name_2 = link_name_2
            self.detach_proxy(req)
            log.warning(f'Detached {model_name_1} from {model_name_2}')
        except rospy.ServiceException as e:
            log.error(f'Failed to detach {model_name_1} from {model_name_2}: {str(e)}')


    def spawn_model(self, name, model_name, pose):
        """
        Command that spawns a model in Gazebo
        (not updated to the new API yet)

        :param name: name of the model, ``str``
        :param model_name: name of the model, ``str``
        :param pose: pose of the model, ``Pose``
        :exception rospy.ServiceException: if the service call failed
        """
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            # Get model SDF file
            sdf_file = MODEL[model_name]['sdf_file']
            # Parse XML file
            root = parse_xml_file(sdf_file)
            model_xml = get_xml_content(root)

            self.spawn_sdf_model(name, model_xml, model_name, pose, "world")
            log.debug(f"Successfully spawned model '{model_name}' in Gazebo")
            rospy.loginfo(f"Successfully spawned model '{model_name}' in Gazebo")
        except rospy.ServiceException as e:
            log.error(f"Failed to spawn model '{model_name}' in Gazebo: {str(e)}")
            rospy.logerr(f"Failed to spawn model '{model_name}' in Gazebo: {str(e)}")

    def delete_model(self, model_name):
        """
        Command that deletes a model in Gazebo
        (not updated to the new API yet)

        :param model_name: name of the model, ``str``
        :exception rospy.ServiceException: if the service call failed
        """
        rospy.wait_for_service('/gazebo/delete_model')
        try:
            self.delete_model_proxy(model_name)
            log.warning(f"Successfully deleted model '{model_name}' from Gazebo")
            rospy.loginfo(f"Successfully deleted model '{model_name}' from Gazebo")
        except rospy.ServiceException as e:
            log.error(f"Failed to delete model '{model_name}' from Gazebo: {str(e)}")
            rospy.logerr(f"Failed to delete model '{model_name}' from Gazebo: {str(e)}")

    def set_pose(self, model_name='', pose=None):
        """
        Command that sets the pose of a model in Gazebo
        (not updated to the new API yet)

        :param model_name: name of the model, ``str``
        :param pose: pose of the model, ``Pose``
        :exception rospy.ServiceException: if the service call failed
        """
        # Retrieve the current pose of the model
        model_state = GazeboCommand.get_model_state(model_name=model_name)

        if pose is not None:
            model_state.pose = pose

        # Set model state
        GazeboCommand.set_model_state(model_name=model_name, model_state=model_state)

        log.warning(f'Set {model_name} to pose: {pose}')
        # log.debug(f'Model state: {model_state}')

    def get_model_state(self, model_name=''):
        """
        Command that retrieves the pose of a model in Gazebo
        (not updated to the new API yet)

        :param model_name: name of the model, ``str``
        :return: pose of the model, ``Pose``
        """
        # Retrieve the current pose of the model
        request = GetModelStateRequest()
        request.model_name = model_name
        response = self.get_model_state_proxy(request)
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = response.pose
        model_state.twist = response.twist
        model_state.reference_frame = "world"

        return model_state
    
    def set_model_state(self, model_name='', model_state=None):
        """
        Command that sets the pose of a model in Gazebo
        (not updated to the new API yet)

        :param model_name: name of the model, ``str``
        :param model_state: pose of the model, ``Pose``
        """
        model_state.model_name = model_name

        # Call the service
        self.set_model_state_proxy(model_state)

    def get_link_state(self, model_name=''):
        """
        Command that retrieves the pose of a link in Gazebo
        (not updated to the new API yet)

        :param model_name: name of the model, ``str``
        :return: pose of the model, ``Pose``
        """
        # Retrieve the current pose of the model
        request = GetLinkStateRequest()
        request.link_name = model_name + '::link'
        response = self.get_link_state_proxy(request)
        link_state = LinkState()
        link_state.link_name = model_name + '::link'
        link_state.pose = response.link_state.pose
        link_state.twist = response.link_state.twist
        link_state.reference_frame = "world"

        return link_state

    def set_link_state(self, model_name='', link_name='', link_state=None):
        """
        Command that sets the pose of a link in Gazebo
        (not updated to the new API yet)

        :param model_name: name of the model, ``str``
        :param link_name: name of the link, ``str``
        :param link_state: pose of the model, ``Pose``
        """
        link_state.link_name = model_name + '::' + link_name
        link_state.reference_frame = "world"

        # Call the service
        self.set_link_state_proxy(link_state)

    def get_original_name(self, renamed_model):
        parts = renamed_model.split()
        original_name = " ".join(parts[:-1])
        return original_name

    def get_model_link_name(self, model_name):
        """
        Returns the name of the link of a model
        :param model_name: name of the model, ``str``
        :return: name of the link, ``str``
        """
        return model_name + '_link'
    
    def set_up_lego_scene(self):
        """
        Sets up the LEGO scene in Gazebo
        """
        # Load the JSON content from the file
        with open(LEGO_SCENE, 'r') as file:
            json_data = json.load(file)

        # Extract the list of objects
        for item in json_data:
            name = item['name']
            model_name = item['type']
            pose = Pose()
            pose.position.x = item['pose']['x']
            pose.position.y = item['pose']['y']
            pose.position.z = item['pose']['z']
            pose.orientation.x = item['pose']['rx']
            pose.orientation.y = item['pose']['ry']
            pose.orientation.z = item['pose']['rz']

            # Delete old model
            self.delete_model(model_name=name)

            # Spawn model
            self.spawn_model(name=name, model_name=model_name, pose=pose)


if __name__ == '__main__':
    gazebo_command = GazeboCommand()
    gazebo_command.set_up_lego_scene()