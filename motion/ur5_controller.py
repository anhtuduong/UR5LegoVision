# Import system
import os
import sys
import threading
import distro
import time

# Resolve paths
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

# Ros utils
import rospy as ros
import roslaunch
import rosnode
import rosgraph
import rospkg
from rospy import Time

# Robot utils
import locosim.robot_control.base_controllers.params as conf
from locosim.robot_control.base_controllers.components.controller_manager import ControllerManager
from locosim.robot_control.base_controllers.utils.utils import Utils
from locosim.robot_control.base_controllers.utils.math_tools import Math
from locosim.robot_control.base_controllers.utils.ros_publish import RosPub
from locosim.robot_control.base_controllers.utils.pidManager import PidManager
from locosim.robot_control.base_controllers.utils.common_functions import getRobotModel
from locosim.robot_control.base_controllers.utils.common_functions import plotJoint, plotEndeff
from locosim.robot_control.base_controllers.components.inverse_kinematics.inv_kinematics_pinocchio import robotKinematics

# Other utils
import numpy as np
from numpy import nan
import tf
import pinocchio as pin
from motion.moveit_control import MoveitControl
from motion.utils import list_to_Pose
from utils_ur5.TransformationUtils import TransformationUtils as tf_utils
from utils_ur5.Logger import Logger as log

from motion import kinematics as kin
from motion import kine as kine

# Services and messages
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
from gazebo_msgs.srv import ApplyBodyWrench
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import geometry_msgs.msg
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Float64MultiArray
from ros_impedance_controller.srv import generic_float
from ros_impedance_controller.srv import MoveJoints, MoveTo

# Constants
from constants import *

# ----------- Class UR5Controller ----------- #

class UR5Controller(threading.Thread):
    """
    Class to control the UR5 robot
    """

    def __init__(self, external_conf = None):
        """
        """
        threading.Thread.__init__(self)

        if (external_conf is not None):
            conf.robot_params = external_conf.robot_params

        self.robot_name = ROBOT_NAME

        self.base_offset = np.array([conf.robot_params[self.robot_name]['spawn_x'],
                                     conf.robot_params[self.robot_name]['spawn_y'],
                                     conf.robot_params[self.robot_name]['spawn_z']])
        
        self.u = Utils()
        self.math_utils = Math()
        self.contact_flag = False
        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        #send data to param server
        self.verbose = conf.verbose
        self.use_torque_control = True

        self.real_robot = conf.robot_params[self.robot_name]['real_robot']

        if self.real_robot:
            self.v_des = 0.2
        else:
            self.v_des = 0.6

        self.dt = conf.robot_params[self.robot_name]['dt']

        self.homing_flag = True

        if (conf.robot_params[self.robot_name]['control_type'] == "torque"):
            self.use_torque_control = True
        else:
            self.use_torque_control = False

        if self.use_torque_control and self.real_robot:
            log.error('Cannot use ur5 in torque control mode')
            sys.exit()

        if conf.robot_params[self.robot_name]['gripper_sim']:
            self.gripper = True
            self.grasping = False
        else:
            self.gripper = False

        self.controller_manager = ControllerManager(conf.robot_params[self.robot_name])

        self.world_name = WORLD_NAME

        log.info('UR5 CONTROLLER INITIALIZED ------------------')

    # --------------------------- #
    def startRealRobot(self):
        """
        """
        os.system("killall rviz gzserver gzclient")
        log.warning('STARTING REAL ROBOT')

        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # launch_file = rospkg.RosPack().get_path('ur_robot_driver') + '/launch/ur5e_bringuself.launch'
        # cli_args = [launch_file,
        #             'headless_mode:=true',
        #             'robot_ip:=192.168.0.100',
        #             'kinematics_config:=/home/laboratorio/my_robot_calibration_1.yaml']
        #
        # roslaunch_args = cli_args[1:]
        # roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        # parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        if (not rosgraph.is_master_online()) or (
                "/" + self.robot_name + "/ur_hardware_interface" not in rosnode.get_node_names()):
            log.error('No ur driver found! Check /ur_hardware_interface')
            sys.exit()
            # log.debug_highlight('Launching the ur driver!')
            # parent.start()

        # run rviz
        package = 'rviz'
        executable = 'rviz'
        args = '-d ' + rospkg.RosPack().get_path('ros_impedance_controller') + '/config/operator.rviz'
        node = roslaunch.core.Node(package, executable, args=args)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)

    # --------------------------- #
    def startSimulator(self, world_name = None, use_torque_control = True, additional_args = None, launch_file = None):
        """
        """
        # needed to be able to load a custom world file
        log.debug_highlight('Adding gazebo model path!')
        custom_models_path = CUSTOM_MODELS_PATH
        if os.getenv("GAZEBO_MODEL_PATH") is not None:
            os.environ["GAZEBO_MODEL_PATH"] +=":"+custom_models_path
        else:
            os.environ["GAZEBO_MODEL_PATH"] = custom_models_path

        if launch_file is None:
            launch_file = LAUNCH_PATH

        # clean up previous process
        os.system("killall rosmaster rviz gzserver gzclient")

        if (distro.linux_distribution()[1] == "16.04"):
            log.error("This file only works with distribution from ROS lunar (I.e. ubuntu 17.04 or later) ")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [launch_file,
                    'spawn_x:=' + str(conf.robot_params[self.robot_name]['spawn_x']),
                    'spawn_y:=' + str(conf.robot_params[self.robot_name]['spawn_y']),
                    'spawn_z:=' + str(conf.robot_params[self.robot_name]['spawn_z'])]
        cli_args.append('use_torque_control:=' + str(use_torque_control))
        if additional_args is not None:
            cli_args.extend(additional_args)
        if world_name is not None:
            log.debug_highlight(f'Setting custom model: {str(world_name)}')
            cli_args.append('world_name:=' + str(world_name))

        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        ros.sleep(1.0)
        log.info('SIMULATION STARTED')

    # --------------------------- #
    def loadModelAndPublishers(self, xacro_path = None, additional_urdf_args = None):
        """
        """
        # instantiating objects
        self.ros_pub = RosPub(self.robot_name, only_visual=True)
        self.pub_des_jstate = ros.Publisher("/command", JointState, queue_size=1, tcp_nodelay=True)
        self.pubRobot = ros.Publisher('/ur5/joint_group_pos_controller/command', Float64MultiArray, queue_size=10)

        # freeze base and pause simulation service
        self.reset_world = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_physics_client = ros.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.get_physics_client = ros.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        self.pause_physics_client = ros.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = ros.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_joints_client = ros.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        self.u.putIntoGlobalParamServer("verbose", self.verbose)

        # subscribers
        self.sub_jstate = ros.Subscriber("/ur5/joint_states",
                                         JointState,
                                         callback = self._receive_jstate,
                                         queue_size = 1,
                                         buff_size = 2 ** 24,
                                         tcp_nodelay=True)

        self.apply_body_wrench = ros.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

        if (self.use_torque_control):
            self.pid = PidManager(self.joint_names)

        # Loading a robot model of robot (Pinocchio)
        if xacro_path is None:
            log.debug_highlight("setting default xacro path")
            xacro_path = rospkg.RosPack().get_path(self.robot_name + '_description') + '/urdf/' + self.robot_name + '.xacro'
        else:
            log.debug_highlight(f'loading custom xacro path: {str(xacro_path)}')

        self.robot = getRobotModel(self.robot_name,
                                   generate_urdf = True,
                                   xacro_path = xacro_path,
                                   additional_urdf_args = additional_urdf_args)
        
        self.sub_ftsensor = ros.Subscriber("/ur5/wrench",
                                           WrenchStamped,
                                           callback = self._receive_ftsensor,
                                           queue_size = 1,
                                           tcp_nodelay = True)
        
        self.switch_controller_srv = ros.ServiceProxy("/ur5/controller_manager/switch_controller", SwitchController)
        self.load_controller_srv = ros.ServiceProxy("/ur5/controller_manager/load_controller", LoadController)
        self.zero_sensor = ros.ServiceProxy("/ur5/ur_hardware_interface/zero_ftsensor", Trigger)

        self.controller_manager.initPublishers(self.robot_name)

        # specific publisher for joint_group_pos_controller that publishes only position
        self.pub_reduced_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command",
                                                    Float64MultiArray,
                                                    queue_size = 10)
        
        #  different controllers are available from the real robot and in simulation
        if self.real_robot:
            self.available_controllers = ["joint_group_pos_controller",
                                          "scaled_pos_joint_traj_controller" ]
        else:
            self.available_controllers = ["joint_group_pos_controller",
                                          "pos_joint_traj_controller" ]
            
        self.active_controller = self.available_controllers[0]

        self.broadcaster = tf.TransformBroadcaster()

        # store in the param server to be used from other planners
        self.utils = Utils()
        self.utils.putIntoGlobalParamServer("gripper_sim", self.gripper)

        # publish end effector pose
        self.pub_ee_pose = ros.Publisher("/ur5/ee_pose", geometry_msgs.msg.Pose, queue_size=1, tcp_nodelay=True)

        # start moveit control
        self.moveit = MoveitControl()

        # sevices to move robot arm
        ros.Service('/ur5/move_joints', MoveJoints, self.move_joints_callback)
        ros.Service('/ur5/move_to', MoveTo, self.move_to_callback)
        ros.Service('/ur5/move_gripper', generic_float, self.move_gripper_callback)

    # --------------------------- #
    def initVars(self):

        self.q = np.zeros(self.robot.na)
        self.qd = np.zeros(self.robot.na)
        self.tau = np.zeros(self.robot.na)
        self.q_des =np.zeros(self.robot.na)
        self.qd_des = np.zeros(self.robot.na)
        self.tau_ffwd =np.zeros(self.robot.na)

        self.x_ee = np.zeros(3)
        self.x_ee_des = np.zeros(3)

        self.contactForceW = np.zeros(3)
        self.contactMomentW = np.zeros(3)

        self.time  = 0.

        #log vars
        self.q_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.q_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.qd_des_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.qd_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.tau_ffwd_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.tau_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.x_ee_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.x_ee_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.contactForceW_log = np.empty((3,conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.time_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.log_counter = 0

    # --------------------------- #
    def startupProcedure(self):
        """
        """
        if (self.use_torque_control):
            #set joint pdi gains
            self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp'],
                                 conf.robot_params[self.robot_name]['kd'],
                                 np.zeros(self.robot.na))
        if (self.real_robot):
            self.zero_sensor()

        self.u.putIntoGlobalParamServer("real_robot", self.real_robot)

        log.info('FINISHED STARTUP -- STARTING CONTROLLER')

    # --------------------------- #
    def switch_controller(self, target_controller):
        """
        Activates the desired controller and stops all others from the predefined list above
        """
        log.debug_highlight(f'Available controllers: {self.available_controllers}')
        log.debug_highlight(f'[Controller manager] loading {target_controller}')

        other_controllers = (self.available_controllers)
        other_controllers.remove(target_controller)
        log.debug_highlight(f'[Controller manager] Switching off: {other_controllers}')

        srv = LoadControllerRequest()
        srv.name = target_controller

        self.load_controller_srv(srv)  
        
        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers 
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_controller_srv(srv)
        self.active_controller = target_controller

    # --------------------------- #
    def homing_procedure(self, dt, v_des, q_home, rate):
        """
        """
        # broadcast base world TF
        # self.broadcaster.sendTransform(self.base_offset, (0.0, 0.0, 0.0, 1.0), Time.now(), '/base_link', '/world')
        v_ref = 0.0

        log.info('STARTING HOMING PROCEDURE')

        self.move_joints(dt, v_des, q_home, rate)

        log.info('HOMING PROCEDURE ACCOMPLISHED')

    # --------------------------- #
    def updateKinematicsDynamics(self):
        """
        """
        # q is continuously updated
        # should put neutral base to compute in the base frame
        self.robot.computeAllTerms(self.q, self.qd)
        # joint space inertia matrix
        self.M = self.robot.mass(self.q)
        # bias terms
        self.h = self.robot.nle(self.q, self.qd)
        # gravity terms
        self.g = self.robot.gravity(self.q)
        # compute ee position in the world frame
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        # this is expressed in the base frame
        self.x_ee = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation
        self.w_R_tool0 = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).rotation
        # publish end effector pose
        self.pub_ee_pose.publish(self.get_ee_pose())

        if self.real_robot:
            # zed2_camera_center is the frame where point cloud is generated in REAL robot
            pointcloud_frame = "zed2_camera_center"
        else:
            # left_camera_optical_frame is the frame where point cloud is generated in SIMULATION
            pointcloud_frame = "zed2_left_camera_optical_frame"
        # offset of the camera in the world frame
        self.x_c= self.robot.framePlacement(self.q, self.robot.model.getFrameId(pointcloud_frame)).translation
        self.w_R_c = self.robot.framePlacement(self.q, self.robot.model.getFrameId(pointcloud_frame)).rotation

        # compute jacobian of the end effector in the base or world frame
        # (they are aligned so in terms of velocity they are the same)
        self.J6 = self.robot.frameJacobian(self.q, self.robot.model.getFrameId(frame_name), False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
        # take first 3 rows of J6 cause we have a point contact            
        self.J = self.J6[:3,:] 
        # broadcast base world TF
        # self.broadcaster.sendTransform(self.base_offset, (0.0, 0.0, 0.0, 1.0), Time.now(), '/base_link', '/world')

    # --------------------------- #
    def _receive_jstate(self, msg):
        """
        """
        self.ros_pub.joint_pub.publish(msg)

        for msg_idx in range(len(msg.name)):          
             for joint_idx in range(len(self.joint_names)):
                 if self.joint_names[joint_idx] == msg.name[msg_idx]: 
                     self.q[joint_idx] = msg.position[msg_idx]
                     self.qd[joint_idx] = msg.velocity[msg_idx]
                     self.tau[joint_idx] = msg.effort[msg_idx]

    # --------------------------- #
    def _receive_ftsensor(self, msg):
        """
        """
        contactForceTool0 = np.zeros(3)
        contactMomentTool0 = np.zeros(3)
        contactForceTool0[0] = msg.wrench.force.x
        contactForceTool0[1] = msg.wrench.force.y
        contactForceTool0[2] = msg.wrench.force.z
        contactMomentTool0[0] = msg.wrench.torque.x
        contactMomentTool0[1] = msg.wrench.torque.y
        contactMomentTool0[2] = msg.wrench.torque.z
        self.contactForceW = self.w_R_tool0.dot(contactForceTool0)
        self.contactMomentW = self.w_R_tool0.dot(contactMomentTool0)

    # --------------------------- #
    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            self.q_des_log[:, self.log_counter] = self.q_des
            self.q_log[:,self.log_counter] =  self.q
            self.qd_des_log[:,self.log_counter] =  self.qd_des
            self.qd_log[:,self.log_counter] = self.qd
            self.tau_ffwd_log[:,self.log_counter] = self.tau_ffwd                    
            self.tau_log[:,self.log_counter] = self.tau
            self.x_ee_log[:, self.log_counter] = self.x_ee
            self.x_ee_des_log[:, self.log_counter] = self.x_ee_des
            self.contactForceW_log[:,self.log_counter] =  self.contactForceW
            self.time_log[self.log_counter] = self.time
            self.log_counter+=1

    # --------------------------- #
    def deregister_node(self):
        log.debug_highlight("deregistering nodes")
        self.ros_pub.deregister_node()
        if not self.real_robot:
            os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")
            os.system(" rosnode kill /gzserver /gzclient")

    # --------------------------- #
    def plotStuff(self, time_log):
        plotJoint('position', time_log, self.time_log, self.q_log, self.q_des_log)
        # plotJoint('position', time_log, self.time_log, self.q_log, self.q_des_log, self.qd_log, self.qd_des_log, None, None, self.tau_log,
        #             self.tau_ffwd_log, self.joint_names)
        # plotJoint('torque', time_log, self.time_log, self.q_log, self.q_des_log, self.qd_log, self.qd_des_log, None, None, self.tau_log,
        #         self.tau_ffwd_log, self.joint_names)
    
    # --------------------------- #
    def move_joints_callback(self, req):
        """
        """
        joint_target = req.q_des
        trajectory = self.moveit.get_trajectory(joint_target)

        if trajectory is None:
            log.error('Failed to get trajectory')
            return False
        
        log.debug_highlight(f'Found trajectory. Starting movement')

        for point in trajectory.points:
            q_des = np.array(point.positions)
            self.move_joints(req.dt, req.v_des, q_des, verbose=False)

        log.info(f'Finished movement')
        return True
    
    # --------------------------- #
    # def move_to_callback(self, req): #NOTE: using MoveIt
    #     """
    #     """
    #     pose_target = list_to_Pose(req.pose_target)
    #     trajectory = self.moveit.get_trajectory(pose_target)

    #     if trajectory is None:
    #         log.error('Failed to get trajectory')
    #         return False
        
    #     log.debug_highlight(f'Found trajectory. Starting movement')

    #     for point in trajectory.points:
    #         q_des = np.array(point.positions)
    #         self.move_joints(req.dt, req.v_des, q_des, verbose=False)

    #     log.info(f'Finished movement')
    #     return True

    # --------------------------- #
    def move_to_callback(self, req): #NOTE: using manual IK
        """
        """
        pose_target = list_to_Pose(req.pose_target)
        log.debug(f'Pose target: {pose_target}')

        pos, rotm = tf_utils.pose_to_position_rotation(pose_target) 
        log.debug(f'Position: {pos}')
        log.debug(f'Rotation: {rotm}')

        T = np.identity(4)
        T[:3,:3] = rotm
        T[:3,3] = pos

        base_transform = np.array([
            [1, 0, 0, 0.5],raise ServiceException("service [%s] responded with an error: %s"%(self.resolved_name, str))
rospy.service.ServiceException: service [/ur5/move_to] responded with an error: b'error processing request: math domain error'
            joints = qi[:,i].flatten().tolist()
            # self.publish_point(point)
            self.move_joints(req.dt, req.v_des, joints, verbose=False)
            log.debug(f'Joints: {joints}')
        self.q_des = qf

        rate = ros.Rate(1/req.dt)
        rate.sleep()

        log.info(f'Finished movement')
        return True

    # --------------------------- #
    def publish_point(self, q):
        msg = Float64MultiArray()
        msg.data = q
        msg.data[5] = 0
        msg.data.extend([70, 70])
        self.pubRobot.publish(msg)
        
    
    # --------------------------- #
    def move_gripper_callback(self, req):
        """
        """
        diameter = req.data
        rate = ros.Rate(1/self.dt)
        self.controller_manager.gm.move_gripper(diameter)
        target = self.controller_manager.gm.q_des_gripper

        count = 0
        while True:
            current = self.controller_manager.gm.getDesGripperJoints()
            self.controller_manager.sendReference(self.q_des)
            count += 1
            rate.sleep()
            if np.any(np.abs(target - current) < 0.0001):
                break
            
            # TODO: handle timeout
        
        current = self.controller_manager.gm.getDesGripperJoints()
        log.info(f'Gripper moved: {count} iterations')
        log.debug(f'Gripper target\t{target}')
        log.debug(f'Gripper current\t{current}')
        return True

    # --------------------------- #
    def move_joints(self, dt, v_des, q_des, verbose=True):
        """
        """
        time_start = time.time()
        
        rate = ros.Rate(1/dt)

        # broadcast base world TF
        # self.broadcaster.sendTransform(self.base_offset, (0.0, 0.0, 0.0, 1.0), Time.now(), '/base_link', '/world')
        v_ref = 0.0
        
        if verbose:
            log.debug_highlight(f'Starting movement:')
            log.debug(f'initial joint error = {np.linalg.norm(self.q_des - q_des)}')
            log.debug(f'q = {self.q.T}')
            log.debug(f'velocity = {v_des}')

        self.q_des = np.copy(self.q)

        while True:
            e = q_des - self.q_des
            e_norm = np.linalg.norm(e)
            if (e_norm != 0.0):
                v_ref += 0.005 * (v_des - v_ref)
                self.q_des += dt * v_ref * e / e_norm
                self.controller_manager.sendReference(self.q_des)
            rate.sleep()
            if (e_norm < 0.001):
                break
        
        if verbose:
            log.info(f'Finished movement in {time.time() - time_start:.2f} seconds')

        rate.sleep()

    # --------------------------- #
    def get_ee_pose(self):
        """
        """
        ee_pose = geometry_msgs.msg.Pose()
        ee_pose.position.x = self.x_ee[0]
        ee_pose.position.y = self.x_ee[1]
        ee_pose.position.z = self.x_ee[2]

        # Create a 4x4 transformation matrix with the rotation matrix and zero translation
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = self.w_R_tool0
        # Convert the transformation matrix to a quaternion
        rot = tf.transformations.quaternion_from_matrix(transformation_matrix)
        ee_pose.orientation.x = rot[0]
        ee_pose.orientation.y = rot[1]
        ee_pose.orientation.z = rot[2]
        ee_pose.orientation.w = rot[3]
        return ee_pose
    
    
