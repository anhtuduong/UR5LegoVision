# -*- coding: utf-8 -*-
"""
Created on 3 May  2022

@author: mfocchi
"""

from __future__ import print_function

import os
import rospy as ros
import sys
# messages for topic subscribers
from docutils.nodes import label
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench, Point
from std_srvs.srv import Trigger

# ros utils
import roslaunch
import rosnode
import rosgraph
import rospkg
from rospy import Time

#other utils
from base_controllers.utils.math_tools import *
from numpy import nan
import pinocchio as pin
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from six.moves import input # solves compatibility issue bw pyuthon 2.x and 3 for raw input that does exists in python 3
from termcolor import colored
import matplotlib.pyplot as plt
from base_controllers.utils.common_functions import plotJoint, plotAdmittanceTracking, plotEndeff

import  base_controllers.params as conf
import L8_conf as lab_conf
robotName = "ur5"

# controller manager management
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib

from base_controllers.components.obstacle_avoidance.obstacle_avoidance import ObstacleAvoidance
from base_controllers.base_controller_fixed import BaseControllerFixed
from base_controllers.components.admittance_controller import AdmittanceControl
from base_controllers.components.controller_manager import ControllerManager
from geometry_msgs.msg import Pose

import tf
from rospy import Time
import time

class LabAdmittanceController(BaseControllerFixed):
    
    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)
        self.real_robot = conf.robot_params[self.robot_name]['real_robot']
        self.homing_flag = self.real_robot
        if (conf.robot_params[self.robot_name]['control_type'] == "torque"):
            self.use_torque_control = 1
        else:
            self.use_torque_control = 0

        if (lab_conf.obstacle_avoidance):
            self.world_name = 'tavolo_obstacles.world'
            if (not self.use_torque_control):
                print(colored("ERRORS: you can use obstacle avoidance only on torque control mode", 'red'))
                sys.exit()
        else:
            #self.world_name = 'tavolo_brick.world'
            #self.world_name = 'palopoli.world'
            self.world_name = None

        if lab_conf.admittance_control and ((not self.real_robot) and (not self.use_torque_control)):
            print(colored("ERRORS: you can use admittance control only on torque control mode or in real robot (need contact force estimation or measurement)", 'red'))
            sys.exit()

        if self.use_torque_control and self.real_robot:
            print(colored(
                "ERRORS: unfortunately...you cannot use ur5 in torque control mode, talk with your course coordinator to buy a better robot...:))",
                'red'))
            sys.exit()

        if conf.robot_params[self.robot_name]['gripper_sim']:
            self.gripper = True
        else:
            self.gripper = False
        self.controller_manager = ControllerManager(conf.robot_params[self.robot_name])

            
        print("Initialized L8 admittance  controller---------------------------------------------------------------")

    def startRealRobot(self):
        os.system("killall  rviz gzserver gzclient")
        print(colored('------------------------------------------------ROBOT IS REAL!', 'blue'))

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = rospkg.RosPack().get_path('ur_robot_driver') + '/launch/ur5e_bringup.launch'
        cli_args = [launch_file,
                    'headless_mode:=true',
                    'robot_ip:=192.168.0.100',
                    'kinematics_config:=/home/laboratorio/my_robot_calibration_1.yaml']

        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        if (not rosgraph.is_master_online()) or ("/" + self.robot_name + "/ur_hardware_interface" not in rosnode.get_node_names()):
            print(colored('ERROR: You should first launch the ur driver!', 'red'))
            sys.exit()
            #print(colored('Launching the ur driver!', 'blue'))
            #parent.start()


        # run rviz
        package = 'rviz'
        executable = 'rviz'
        args = '-d ' + rospkg.RosPack().get_path('ros_impedance_controller') + '/config/operator.rviz'
        node = roslaunch.core.Node(package, executable, args=args)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)



    def loadModelAndPublishers(self, xacro_path):
        super().loadModelAndPublishers(xacro_path)

        self.sub_ftsensor = ros.Subscriber("/" + self.robot_name + "/wrench", WrenchStamped,
                                           callback=self._receive_ftsensor, queue_size=1, tcp_nodelay=True)
        self.switch_controller_srv = ros.ServiceProxy(
            "/" + self.robot_name + "/controller_manager/switch_controller", SwitchController)
        self.load_controller_srv = ros.ServiceProxy("/" + self.robot_name + "/controller_manager/load_controller",
                                                    LoadController)

        self.zero_sensor = ros.ServiceProxy("/" + self.robot_name + "/ur_hardware_interface/zero_ftsensor", Trigger)

        self.pub_ee_pose = ros.Publisher("/" + self.robot_name + "/ee_pose", Pose, queue_size=10)
        self.controller_manager.initPublishers(self.robot_name)
        #  different controllers are available from the real robot and in simulation
        if self.real_robot:
            self.available_controllers = [
                "joint_group_pos_controller",
                "scaled_pos_joint_traj_controller" ]
        else:
            self.available_controllers = ["joint_group_pos_controller",
                                          "pos_joint_traj_controller" ]
        self.active_controller = self.available_controllers[0]

        self.broadcaster = tf.TransformBroadcaster()

    def applyForce(self):
        wrench = Wrench()
        wrench.force.x = 0
        wrench.force.y = 0
        wrench.force.z = 30
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0
        reference_frame = "world" # you can apply forces only in this frame because this service is buggy, it will ignore any other frame
        reference_point = Point(x = 0, y = 0, z = 0)
        try:
            self.apply_body_wrench(body_name="ur5::wrist_3_link", reference_frame=reference_frame, reference_point=reference_point , wrench=wrench, duration=ros.Duration(10))
        except:
            pass

    def _receive_ftsensor(self, msg):
        contactForceTool0 = np.zeros(3)
        contactMomentTool0 = np.zeros(3)
        contactForceTool0[0] = msg.wrench.force.x
        contactForceTool0[1] = msg.wrench.force.y
        contactForceTool0[2] = msg.wrench.force.z
        contactMomentTool0[0] = msg.wrench.torque.x
        contactMomentTool0[1] = msg.wrench.torque.y
        contactMomentTool0[2] = msg.wrench.torque.z
        self.contactForceW = self.w_R_tool0_without_gripper.dot(contactForceTool0)
        self.contactMomentW = self.w_R_tool0_without_gripper.dot(contactMomentTool0)

    def deregister_node(self):
        print( "deregistering nodes"     )
        self.ros_pub.deregister_node()
        if not self.real_robot:
            os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")
            os.system(" rosnode kill /gzserver /gzclient")
                                                                                                                                     
    def updateKinematicsDynamics(self):
        # q is continuously updated
        # to compute in the base frame  you should put neutral base
        self.robot.computeAllTerms(self.q, self.qd)
        # joint space inertia matrix
        self.M = self.robot.mass(self.q)
        # bias terms
        self.h = self.robot.nle(self.q, self.qd)
        #gravity terms
        self.g = self.robot.gravity(self.q)
        #compute ee position  in the world frame

        # this is expressed in a workdframe with the origin attached to the base frame origin
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        self.w_R_tool0 = self.robot.framePlacement(self.q, self.robot.model.getFrameId(conf.robot_params[self.robot_name]['ee_frame'])).rotation
        self.w_R_tool0_without_gripper = self.robot.framePlacement(self.q, self.robot.model.getFrameId('tool0_without_gripper')).rotation
        self.x_ee = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation

        # compute jacobian of the end effector in the world frame
        self.J6 = self.robot.frameJacobian(self.q, self.robot.model.getFrameId(frame_name), False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
        # take first 3 rows of J6 cause we have a point contact            
        self.J = self.J6[:3,:] 
        #compute contact forces                        
        self.estimateContactForces()
        # broadcast base world TF
        self.broadcaster.sendTransform(self.base_offset, (0.0, 0.0, 0.0, 1.0), Time.now(), '/base_link', '/world')

    def estimateContactForces(self):  
        # estimate ground reaction forces from torques tau
        if self.use_torque_control:
            self.contactForceW = np.linalg.inv(self.J6.T).dot(self.h-self.tau)[:3]
                                 
    def startupProcedure(self):
        if (self.use_torque_control):
            #set joint pdi gains
            self.pid.setPDjoints( conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'], np.zeros(self.robot.na))
            #only torque loop
            #self.pid.setPDs(0.0, 0.0, 0.0)
        if (self.real_robot):
            self.zero_sensor()
        print(colored("finished startup -- starting controller", "red"))
        
    def initVars(self):
        super().initVars()

        # log variables relative to admittance controller
        self.q_des_adm_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.x_ee_des_adm_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.EXTERNAL_FORCE = False
        self.payload_weight_avg = 0.0
        self.polynomial_flag = False
        self.obs_avoidance = ObstacleAvoidance()
        # position of the center of the objects is in WF
        self.obs_avoidance.setCubeParameters(0.25, np.array([0.125, 0.75,0.975]))
        self.obs_avoidance.setCylinderParameters(0.125, 0.3, np.array([0.6, 0.25, 1.0]))
        self.admit = AdmittanceControl(self.ikin, lab_conf.Kx, lab_conf.Dx, conf.robot_params[self.robot_name])


        if lab_conf.USER_TRAJECTORY:
            self.Q_ref = []
            for name in lab_conf.traj_file_name:
                data = np.load(lab_conf.traj_folder + name + '.npz')
                self.Q_ref.append(data['q'])


    def logData(self):
        if (conf.robot_params[self.robot_name]['control_type'] == "admittance"):
            self.q_des_adm_log[:, self.log_counter] = self.q_des_adm
            self.x_ee_des_adm_log[:, self.log_counter] = self.x_ee_des_adm
        # I need to do this after because it updates log counter
        super().logData()

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        print('Available controllers: ',self.available_controllers)
        print('Controller manager: loading ', target_controller)

        other_controllers = (self.available_controllers)
        other_controllers.remove(target_controller)
        print('Controller manager:Switching off  :  ',other_controllers)

        srv = LoadControllerRequest()
        srv.name = target_controller

        self.load_controller_srv(srv)  
        
        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers 
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_controller_srv(srv)
        self.active_controller = target_controller

    def send_ee_pose(self):
        msg = Pose()
        msg.position = self.x_ee + self.base_offset
        self.pub_ee_pose.publish(msg)

    def send_joint_trajectory(self):
        # Creates a trajectory and sends it using the selected action server
        trajectory_client = actionlib.SimpleActionClient("{}/follow_joint_trajectory".format("/" + self.robot_name + "/"+self.active_controller), FollowJointTrajectoryAction)
        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        # The following list are arbitrary positions
        # Change to your own needs if desired q0 [ 0.5, -0.7, 1.0, -1.57, -1.57, 0.5]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
        print(colored("JOINTS ARE: ", 'blue'), self.q.transpose())
        # position_list = [[0.5, -0.7, 1.0, -1.57, -1.57, 0.5]]  # limits([0,-pi], [-pi/2,pi/2],  [0, -pi])
        # position_list.append([0.5, -0.7 - 0.2, 1.0 - 0.1, -1.57, -1.57, 0.5])
        # position_list.append([0.5 + 0.5, -0.7 - 0.3, 1.0 - 0.1, -1.57, -1.57, 0.5])
        # position_list.append([0.5 + 0.5, -0.7 - 0.3, 1.0 , -1., -1.57, 0.5])

        self.q0 = conf.robot_params[p.robot_name]['q_0']
        dq1 = np.array([0.2, 0,0,0,0,0])
        dq2 = np.array([0.2, -0.2, 0, 0, 0, 0])
        dq3 = np.array([0.2, -0.2, 0.4, 0, 0, 0])
        position_list = [self.q0]  # limits([0,-pi], [-pi/2,pi/2],  [0, -pi])
        position_list.append(self.q0 + dq1)
        position_list.append(self.q0 + dq2)
        position_list.append(self.q0 + dq3)
        print(colored("List of targets for joints: ",'blue'))
        print(position_list[0])
        print(position_list[1])
        print(position_list[2])
        print(position_list[3])

        duration_list = [5.0, 10.0, 20.0, 30.0]
        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = ros.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        self.ask_confirmation(position_list)     
        print("Executing trajectory using the {}".format("pos_joint_traj_controller"))
        trajectory_client.send_goal(goal)

        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        print("Trajectory execution finished in state {}".format(result.error_code))

    def send_joint_trajectory_2(self):
        # Creates a trajectory and sends it using the selected action server
        trajectory_client = actionlib.SimpleActionClient("{}/follow_joint_trajectory".format("/" + self.robot_name + "/"+self.active_controller), FollowJointTrajectoryAction)
        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()

        # The following list are arbitrary positions
        # Change to your own needs if desired q0 [ 0.5, -0.7, 1.0, -1.57, -1.57, 0.5]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
        print(colored("JOINTS ARE: ", 'blue'), self.q.transpose())

        if p.real_robot:
            goal.trajectory.joint_names = self.joint_names
            position_list = [[-0.4253643194781702,  -0.9192648094943543, -2.162015914916992,-1.621634145776266, -1.5201204458819788, -2.2737816015826624]] #go on 1 brick
            position_list.append([-0.42451507249941045,  -0.9235735100558777, -1.975731611251831,-1.8549186191954554, -1.534570042287008, -2.1804688612567347]) # approach 1 brick and grasp
            position_list.append([-0.2545421759234827, -1.2628285449794312, -2.049499988555908,  -1.3982257705977936, -1.4819391409503382, -2.4832173029529017] ) # move 2 second brick
            position_list.append([-0.2545355002032679,  -1.2625364822200318,  -1.910099983215332, -1.5169030030122777, -1.4750459829913538, -2.4462133089648646] ) # approach 2 brick and release
            position_list.append([ -0.2544291655169886,-1.277967320089676, -2.1508238315582275,  -1.2845929724029084, -1.465815846120016, -2.445918385182516]) # evade and open gripper

            # print(colored("List of targets for joints: ",'blue'))
            # print(position_list[0])
            # print(position_list[1])
            # print(position_list[2])
            # print(position_list[3])
            # print(position_list[4])

            duration_list = [5.0, 5.0, 5.0, 5.0, 5]
            gripper_state = ['open', 'close', 'idle', 'open', 'open']
            gripper_diameter = [130, 45, 45, 65, 130]
            self.ask_confirmation(position_list)

            # set a different goal with gripper closed or opened
            for i, position in enumerate(position_list):
                point = JointTrajectoryPoint()
                point.positions = position
                point.time_from_start = ros.Duration(duration_list[i])
                # add a single goal and execute it
                goal.trajectory.points = [point]

                print("reaching position: ", position)
                trajectory_client.send_goal(goal)
                trajectory_client.wait_for_result()
                # while not trajectory_client.get_state() == 3:
                #     ros.sleep(1)
                #     print("reaching position: ", position)
                result = trajectory_client.get_result()
                print("Target Reached error code {}".format(result.error_code))
                if (gripper_state[i] == 'close'):
                    print("closing gripper")
                    p.controller_manager.gm.move_gripper(gripper_diameter[i])
                if (gripper_state[i] == 'open'):
                    print("opening gripper")
                    p.controller_manager.gm.move_gripper(gripper_diameter[i])
                time.sleep(2.)
        else: # simulation need to manage gripper as desjoints
            goal.trajectory.joint_names = self.joint_names + ['hand_1_joint', 'hand_2_joint', 'hand_3_joint']
            position_list = [conf.robot_params[p.robot_name]['q_0'].tolist() +
                             [self.controller_manager.gm.mapToGripperJoints(40), self.controller_manager.gm.mapToGripperJoints(40), self.controller_manager.gm.mapToGripperJoints(40)] ] # go home
            position_list.append( [-0.4253643194781702, -0.9192648094943543, -2.162015914916992, -1.621634145776266, -1.5201204458819788, -2.2737816015826624, self.controller_manager.gm.mapToGripperJoints(110), self.controller_manager.gm.mapToGripperJoints(110), self.controller_manager.gm.mapToGripperJoints(110)])  # go on 1 brick
            position_list.append([-0.42451507249941045, -0.9235735100558777, -1.975731611251831, -1.8549186191954554, -1.534570042287008, -2.1804688612567347, self.controller_manager.gm.mapToGripperJoints(110), self.controller_manager.gm.mapToGripperJoints(110), self.controller_manager.gm.mapToGripperJoints(110)])  # approach 1 brick
            position_list.append( [-0.42451507249941045, -0.9235735100558777, -1.975731611251831, -1.8549186191954554, -1.534570042287008,  -2.1804688612567347, self.controller_manager.gm.mapToGripperJoints(55), self.controller_manager.gm.mapToGripperJoints(65), self.controller_manager.gm.mapToGripperJoints(65)])  # close gripper
            position_list.append( [-0.2545421759234827, -1.2628285449794312, -2.049499988555908, -1.3982257705977936, -1.4819391409503382, -2.4832173029529017,self.controller_manager.gm.mapToGripperJoints(55), self.controller_manager.gm.mapToGripperJoints(65), self.controller_manager.gm.mapToGripperJoints(65)])  # move 2 second brick
            position_list.append( [-0.2545355002032679, -1.2625364822200318, -1.910099983215332, -1.5169030030122777, -1.4750459829913538, -2.4462133089648646,self.controller_manager.gm.mapToGripperJoints(55), self.controller_manager.gm.mapToGripperJoints(65), self.controller_manager.gm.mapToGripperJoints(65)])  # approach 2 brick
            position_list.append( [-0.2545355002032679, -1.2625364822200318, -1.910099983215332, -1.5169030030122777, -1.4750459829913538, -2.4462133089648646,self.controller_manager.gm.mapToGripperJoints(65), self.controller_manager.gm.mapToGripperJoints(90), self.controller_manager.gm.mapToGripperJoints(90)])  # open gripper
            position_list.append( [-0.2544291655169886, -1.277967320089676, -2.1508238315582275, -1.2845929724029084, -1.465815846120016,  -2.445918385182516, self.controller_manager.gm.mapToGripperJoints(65), self.controller_manager.gm.mapToGripperJoints(90), self.controller_manager.gm.mapToGripperJoints(90)])  # evade
            position_list.append( [-0.2544291655169886, -1.277967320089676, -2.1508238315582275, -1.2845929724029084, -1.465815846120016,  -2.445918385182516, self.controller_manager.gm.mapToGripperJoints(130), self.controller_manager.gm.mapToGripperJoints(130), self.controller_manager.gm.mapToGripperJoints(130)])  # open gripper

            duration_list = [5., 5.0, 5.0, 2., 5.0, 5.0, 2., 5., 2.]
            self.ask_confirmation(position_list)

            # set a different goal with gripper closed or opened
            for i, position in enumerate(position_list):
                point = JointTrajectoryPoint()
                point.positions = position
                point.time_from_start = ros.Duration(duration_list[i])
                # add a single goal and execute it
                goal.trajectory.points = [point]
                print("reaching position: ", position)
                trajectory_client.send_goal(goal)
                trajectory_client.wait_for_result()
                result = trajectory_client.get_result()
                print("Target Reached error code {}".format(result.error_code))

    def ask_confirmation(self, waypoint_list):
        """Ask the user for confirmation. This function is obviously not necessary, but makes sense
        in a testing script when you know nothing about the user's setup."""
        ros.logwarn("The robot will move to the following waypoints: \n{}".format(waypoint_list))
        confirmed = False
        valid = False
        while not valid:
            input_str = input(
                "Please confirm that the robot path is clear of obstacles.\n"
                "Keep the EM-Stop available at all times. You are executing\n"
                "the motion at your own risk. Please type 'y' to proceed or 'n' to abort: " )
            valid = input_str in ["y", "n"]
            if not valid:
                ros.loginfo("Please confirm by entering 'y' or abort by entering 'n'")
            else:
                if (input_str == "y"):
                    confirmed = True
        if not confirmed:
            ros.loginfo("Exiting as requested by user.")
            sys.exit(0)

    def deregister_node(self):
        super().deregister_node()
        if not self.real_robot:
            os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")
            os.system(" rosnode kill /gzserver /gzclient")

    def plotStuff(self):
        if not (conf.robot_params[p.robot_name]['control_mode'] == "trajectory"):
            if (lab_conf.admittance_control):
                plotJoint('position', 0, self.time_log, self.q_log, self.q_des_log, self.qd_log, self.qd_des_log, None, None, self.tau_log,
                          self.tau_ffwd_log, self.joint_names, self.q_des_adm_log)
                plotAdmittanceTracking(3, self.time_log, self.x_ee_log, self.x_ee_des_log, self.x_ee_des_adm_log, self.contactForceW_log)
            else:
                plotJoint('position', 0, self.time_log, self.q_log, self.q_des_log, self.qd_log, self.qd_des_log, None, None, self.tau_log,
                          self.tau_ffwd_log, self.joint_names, self.q_des_log)
            plotJoint('torque', 2, self.time_log, self.q_log, self.q_des_log, self.qd_log, self.qd_des_log, None, None, self.tau_log,
                      self.tau_ffwd_log, self.joint_names)
            plotEndeff('force', 1, p.time_log, p.contactForceW_log)


def talker(p):
    p.start()
    if p.real_robot:
        p.startRealRobot()
    else:
        additional_args = ['gripper:=' + str(p.gripper)]
        p.startSimulator(p.world_name, p.use_torque_control, additional_args)

    # specify xacro location
    xacro_path = rospkg.RosPack().get_path('ur_description') + '/urdf/' + p.robot_name + '.urdf.xacro'
    p.loadModelAndPublishers(xacro_path)
    p.initVars()
    p.startupProcedure()
    # sleep to avoid that the real robot crashes on the table
    time.sleep(3.)
    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    p.q_des = np.copy(p.q_des_q0)
    p.admit.setPosturalTask(np.copy(p.q_des_q0))


    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    if (conf.robot_params[p.robot_name]['control_mode'] == "trajectory"):
        # to test the trajectory
        if (p.real_robot):
            p.switch_controller("scaled_pos_joint_traj_controller")
        else:
            p.switch_controller("pos_joint_traj_controller")
        p.send_joint_trajectory_2()

    else:
        if not p.use_torque_control:            
            p.switch_controller("joint_group_pos_controller")
        # reset to actual
        p.updateKinematicsDynamics()
        p.time_poly = None

        ext_traj_counter = 0    # counter for which trajectory is currently tracked
        ext_traj_t = 0          # counter for the time inside a trajectory
        traj_completed = False

        gripper_on = 0
        #control loop
        while not ros.is_shutdown():
            # homing procedure
            if p.homing_flag:
                dt = conf.robot_params[p.robot_name]['dt']
                v_des = lab_conf.v_des_homing
                v_ref = 0.0
                print(colored("STARTING HOMING PROCEDURE",'red'))
                q_home = conf.robot_params[p.robot_name]['q_0']
                p.q_des = np.copy(p.q)
                print("Initial joint error = ", np.linalg.norm(p.q_des - q_home))
                print("q = ", p.q.T)
                print("Homing v des", v_des)
                while True:
                    e = q_home - p.q_des
                    e_norm = np.linalg.norm(e)
                    if(e_norm!=0.0):
                        v_ref += 0.005*(v_des-v_ref)
                        p.q_des += dt*v_ref*e/e_norm
                        p.controller_manager.sendReference(p.q_des)
                    rate.sleep()
                    if (e_norm<0.001):
                        p.homing_flag = False
                        print(colored("HOMING PROCEDURE ACCOMPLISHED", 'red'))
                        p.controller_manager.gm.move_gripper(100)
                        print(colored("GRIPPER CLOSED", 'red'))
                        break

            #update the kinematics
            p.updateKinematicsDynamics()

            if lab_conf.USER_TRAJECTORY:
                if (int(ext_traj_t) < p.Q_ref[ext_traj_counter].shape[0]): # and p.time>6.0:
                    p.q_des = p.Q_ref[ext_traj_counter][int(ext_traj_t),:]
                    ext_traj_t += 1.0/lab_conf.traj_slow_down_factor
                else:
                    if(ext_traj_counter < len(p.Q_ref)-1):
                        print(colored("TRAJECTORY %d COMPLETED"%ext_traj_counter, 'blue'))
                        if(ext_traj_counter==0):
                            p.controller_manager.gm.move_gripper(65)
                        if (ext_traj_counter == 1):
                            p.controller_manager.gm.move_gripper(30)
                        ext_traj_counter += 1
                        ext_traj_t = 0
                    elif(not traj_completed):
                        print(colored("LAST TRAJECTORY COMPLETED", 'red'))
                        p.controller_manager.gm.move_gripper(60)
                        traj_completed = True
                        #ext_traj_t = 0
                        #ext_traj_counter = 0

            # EXE L8-1.1: set constant joint reference
            #p.q_des = np.copy(p.q_des_q0)

            # test gripper
            # if p.gripper:
            #     if p.time > 5.0 and (gripper_on == 0):
            #         print("gripper 30")
            #         p.controller_manager.gm.move_gripper(30)
            #         gripper_on = 1
            #     if (gripper_on == 1) and p.time > 10.0:
            #         print("gripper 100")
            #         p.controller_manager.gm.move_gripper(100)
            #         gripper_on = 2

            # EXE L8-1.2: set sinusoidal joint reference
            # p.q_des  = p.q_des_q0  + lab_conf.amplitude * np.sin(2*np.pi*lab_conf.frequency*p.time)


            # EXE L8-1.3: set constant ee reference
            # p.x_ee_des = lab_conf.ee_reference
            # p.ros_pub.add_marker(p.x_ee_des + p.base_offset, color='blue')
            # p.q_des, ok, out_ws = p.ikin.endeffectorInverseKinematicsLineSearch(p.x_ee_des,
            #                                                                     conf.robot_params[p.robot_name][
            #                                                                         'ee_frame'], p.q, False, False,
            #                                                                     postural_task=True, w_postural=0.00001,
            #                                                                     q_postural=p.q_des_q0)

            # EXE L8-1.5:  set constant ee reference and create polynomial trajectory to reach it
            # p.x_ee_des = lab_conf.ee_reference
            # p.ros_pub.add_marker(p.x_ee_des + p.base_offset, color='blue')
            # if not p.polynomial_flag and p.time > 3.0:
            #     print(colored("STARTING POLYNOMIAL",'red'))
            #     p.time_poly = p.time
            #     p.x_intermediate = np.zeros(3)
            #     a = np.empty((3, 6))
            #     for i in range(3):
            #         a[i, :] = coeffTraj( lab_conf.poly_duration, p.x_ee[i], p.x_ee_des[i])
            #     p.polynomial_flag = True
            # # Polynomial trajectory for x,y,z coordinates
            # if  p.polynomial_flag and (p.time - p.time_poly) <  lab_conf.poly_duration:
            #     t = p.time - p.time_poly
            #     for i in range(3):
            #         p.x_intermediate[i] = a[i, 0] + a[i,1] * t + a[i,2] * pow(t, 2) + a[i,3] * pow(t, 3) + a[i,4]*pow(t, 4) + a[i,5]*pow(t, 5)
            # if p.polynomial_flag:
            #     p.q_des, ok, out_ws = p.ikin.endeffectorInverseKinematicsLineSearch(p.x_intermediate,  conf.robot_params[p.robot_name]['ee_frame'], p.q, False, False,
            #                                                                     postural_task=True, w_postural=0.00001,
            #                                                                     q_postural=p.q_des_q0)

            # EXE L8-1.4:  set constant ee reference and desired orientation
            # rpy_des = lab_conf.des_orient
            # w_R_e_des = p.math_utils.eul2Rot(rpy_des) # compute rotation matrix representing the desired orientation from Euler Angles
            # p.q_des, ok, out_ws = p.ikin.endeffectorFrameInverseKinematicsLineSearch(p.x_ee_des, w_R_e_des, conf.robot_params[p.robot_name]['ee_frame'], p.q)
            # TODO create polynomial for orientation!

            # EXE L8-2 - admittance control
            if (lab_conf.admittance_control):
                p.EXTERNAL_FORCE = True
                ## TODO to implement at the velocity level you need to load the VelocityInterface and use the joint_group_vel_controller
                p.x_ee_des = p.robot.framePlacement(p.q_des,p.robot.model.getFrameId(conf.robot_params[p.robot_name]['ee_frame'])).translation
                p.q_des_adm, p.x_ee_des_adm = p.admit.computeAdmittanceReference(p.contactForceW, p.x_ee_des, p.q)

            # EXE L8-2.5 - load estimation
            # if (p.time > 10.0):
            #     p.payload_weight_avg = 0.99 * p.payload_weight_avg + 0.01 * (-p.contactForceW[2] / 9.81)
            #     print("estimated load: ", p.payload_weight_avg)

            # controller with gravity coriolis comp
            p.tau_ffwd = p.h + np.zeros(p.robot.na)

            # only torque loop (not used)
            #p.tau_ffwd = conf.robot_params[p.robot_name]['kp']*(np.subtract(p.q_des,   p.q))  - conf.robot_params[p.robot_name]['kd']*p.qd

            # override the position command if you use admittance controller
            if (p.time > 1.5) and (lab_conf.admittance_control):  # activate admittance control only after a few seconds to allow initialization
                q_to_send = p.q_des_adm
            else:
                q_to_send = p.q_des

            if (lab_conf.obstacle_avoidance):
                p.tau_ffwd = p.obs_avoidance.computeTorques(p,  lab_conf.des_ee_goal)
            # send commands to gazebo
            p.controller_manager.sendReference(q_to_send, p.qd_des, p.tau_ffwd)

            if(not lab_conf.obstacle_avoidance):
                p.ros_pub.add_arrow(p.x_ee + p.base_offset, p.contactForceW / (6 * p.robot.robot_mass), "green")

            # log variables
            if (p.time > 1.0):
                p.logData()

            # disturbance force
            if (p.time > 10.0 and p.EXTERNAL_FORCE):
                p.applyForce()
                p.EXTERNAL_FORCE = False

            # plot end-effector
            p.ros_pub.add_marker(p.x_ee + p.base_offset)
            p.ros_pub.publishVisual()
            p.send_ee_pose()

            #wait for synconization of the control loop
            rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),  3)  # to avoid issues of dt 0.0009999



if __name__ == '__main__':

    p = LabAdmittanceController(robotName)

    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        if   conf.plotting:
            p.plotStuff()

    
        
