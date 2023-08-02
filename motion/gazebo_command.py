
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


class GazeboCommand():

    def __init__(self):
        """
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
        
    # def spawn_model_static(self, model_name, is_static=True):
    #     """
    #     """
    #     # Get model original name
    #     original_model_name = self.get_original_name(model_name)
    #     # Get model SDF file
    #     sdf_file = MODEL[original_model_name]['sdf_file']
    #     # Parse XML file
    #     root = parse_xml_file(sdf_file)
    #     # Find static element
    #     element = find_element(root, 'static')

    #     if is_static:
    #         # Make static
    #         modify_element(element, '1')
    #     else:
    #         # Make non static
    #         modify_element(element, '0')
        
    #     # Get model current pose
    #     model_state = self.get_model_state(model_name=model_name)
    #     # Spawn modified model in Gazebo
    #     new_model_name = model_name + ' static'
    #     self.spawn_model(model_name=new_model_name, model_xml=get_xml_content(root), pose=model_state.pose)

    #     if is_static:
    #         log.warning(f'Spawned model {model_name} static')
    #     else:
    #         log.warning(f'Spawned model {model_name} non static')


    def spawn_model(self, name, model_name, pose):
        """
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
        """
        model_state.model_name = model_name

        # Call the service
        self.set_model_state_proxy(model_state)

    def get_link_state(self, model_name=''):
        """
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
        """
        return model_name + '_link'
    
    def set_up_lego_scene(self):
        """
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