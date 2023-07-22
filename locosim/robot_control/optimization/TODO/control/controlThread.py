import numpy as np
from scipy.linalg import block_diag
import rospy as ros
import threading
import math as pymath

import roslaunch
import rospkg
import rosnode

from control.controlRoutines import *
from tools.mathutils import euler_from_quaternion  # taken from
from tools.utils import Utils
from tools.math_tools import Math
from tools.plottingFeatures import *
from tools.ros_publish import RosPub
from tools.pidManager import PidManager
from tools.plottingFeatures import plotReference

# gazebo / ros messages
from std_srvs.srv import Empty, EmptyRequest
from std_srvs.srv import Empty
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.msg import ModelState
#gazebo services
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsPropertiesRequest, GetPhysicsPropertiesRequest
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ODEPhysics
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

from termcolor import colored

#TODO
#import pinocchio as pin
#import example_robot_data


class ControlThread(threading.Thread):
    def __init__(self):

        threading.Thread.__init__(self)



        self.grForcesW = np.zeros(12)
        self.WBcontrForcesW = np.zeros(12)
        self.basePoseW = np.zeros(6)
        self.comPoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)
        self.comTwistW = np.zeros(6)
        self.q = np.zeros(12)
        self.qd = np.zeros(12)
        self.q_des = np.array([-0.2, 0.75, -1.5, -0.2, 0.75, -1.5, -0.2, -0.75, 1.5, -0.2, -0.75, 1.5])
        self.qd_des = np.zeros(12)
        self.tau_ffwd = np.zeros(12)
        self.tau = np.zeros(12)
        self.h_joint = np.zeros(12)
        self.B_offCoM = np.array([0.0158118, 0.000230555, -0.0433619])

        self.J = [np.eye(3)]* 4
        self.wJ = [np.eye(3)]* 4


        self.sim_time = 0.0
        self.numberOfReceivedMessages = 0
        self.numberOfPublishedMessages = 0
        self.b_R_w = np.eye(3)
        self.robotMass = 0
        self.desLinVelocity = np.array([0.0, 0.0, 0.0])
        self.desHeadingVelocity = 0.0
        self.des_com_twist_old = np.zeros(6)
        self.u = Utils()

        
        self.firstTime = True
        timer = 0.0
        self.actual_feetW = [np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1))]
        self.des_feetW = [np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1))]
        self.ref_feetW = np.zeros(12)
        self.J = [np.eye(3)] * 4
        self.mathJet = Math()

        #plot legends
        self.legend_desired = 'Desired'
        self.legend_actual = 'Actual'
        self.legend_ref = 'Ref'

        #pinocchio stuff (TODO)
        # self.model = example_robot_data.loadHyQ().model
        # self.data = model.createData()

    def run(self):
        pass

    def loadConfig(self, optiParam):
        # instantiate graphic utils (launches reference generator node also)
        self.ros_pub = RosPub()

        # Define data

        self.nx = optiParam.nx
        self.nu = optiParam.nu

        self.desLinVelocity[0] = optiParam.desired_velocity_x
        self.desLinVelocity[1] = optiParam.desired_velocity_y
        self.desHeadingVelocity = optiParam.desHeadingVelocity

        self.step_height = 0.1
        self.step_length = np.array([0.0, 0.0, 0.0, 0.0])
        self.delta_step = [np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1)), np.zeros((3, 1))]
        self.swing_time = np.array([0.0, 0.0, 0.0, 0.0])
        self.sampleFootW = np.zeros((4, 3))
        self.stance_legs = np.array([True, True, True, True])
        self.swinging_legs = np.array([False, False, False, False])
        self.foot_ball_correction = np.array([0.0, 0.0, -0.02155])

        self.robotMass = optiParam.robotMass
        self.robotHeight = optiParam.robotHeight
        self.robotInertia = optiParam.robotInertia
        self.cycle_time = optiParam.cycle_time
        self.gait_type = optiParam.gait_type
        self.duty_factor = optiParam.duty_factor
        self.offset_a = optiParam.offset_a
        self.offset_b = optiParam.offset_b
        self.offset_c = optiParam.offset_c
        self.offset_d = optiParam.offset_d

        self.pallet_location = optiParam.pallet_location
        self.pallet_height = optiParam.pallet_height
        self.haptic_td = optiParam.haptic_td

        # Prediction horizon same for reference
        self.prediction_horizon = optiParam.prediction_horizon

        # replanning window
        self.replanning_window = optiParam.replanning_window

        # Sampling time for planner
        self.time_resolution = optiParam.time_resolution
        # Control loop duration
        self.control_resolution = optiParam.control_resolution
        # Type of model
        self.model_type = optiParam.model_type
        # Mobility in the cost
        self.include_mobility = optiParam.include_mobility
        # Cone constraints in the cost
        self.include_coneCons = optiParam.include_coneCons
        # Unilaterality constraints in the cost
        self.include_uniCons = optiParam.include_uniCons

        self.solver = optiParam.solver

        # LTV model on/off flag (if 0 then LTI model is used)
        self.include_ltv_model = optiParam.include_ltv_model

        self.verbose = optiParam.verbose

        self.mapper_on = optiParam.mapper_on

        self.Ts = optiParam.time_resolution

        self.replanning_on = optiParam.replanning_on

        self.force_th = optiParam.force_th

        self.controller_type = optiParam.controller_type

        print("loaded Config")

    def _receive_contact(self, msg):
        # ground truth (only works with framwork, dls_hw_sim has already out convention NOT WORKING PROPERLY WITH PALLET!!!)
        # self.grForcesW[0] = msg.states[0].wrenches[0].force.x
        # self.grForcesW[1] = msg.states[0].wrenches[0].force.y
        # self.grForcesW[2] = msg.states[0].wrenches[0].force.z
        # self.grForcesW[3] = msg.states[1].wrenches[0].force.x
        # self.grForcesW[4] = msg.states[1].wrenches[0].force.y
        # self.grForcesW[5] = msg.states[1].wrenches[0].force.z
        # self.grForcesW[6] = msg.states[2].wrenches[0].force.x
        # self.grForcesW[7] = msg.states[2].wrenches[0].force.y
        # self.grForcesW[8] = msg.states[2].wrenches[0].force.z
        # self.grForcesW[9] = msg.states[3].wrenches[0].force.x
        # self.grForcesW[10] = msg.states[3].wrenches[0].force.y
        # self.grForcesW[11] = msg.states[3].wrenches[0].force.z

        # estimate ground reaction forces from tau (TODO check jacs)
        self.estimateContactForces()

    def estimateContactForces(self):
        # estimate ground reaxtion forces from tau (missing h_joints)
        for leg in range(4):
            grf = np.linalg.inv(self.wJ[leg].T).dot(self.u.getLegJointState(leg, - self.tau ))
            self.u.setLegJointState(leg, grf, self.grForcesW)


    def _receive_pose_real_time(self, msg):

        print(msg.pose[1].position.z)

    def _receive_pose(self, msg):
        # These are base pose and twist that is different than COM due to offset
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.basePoseW[self.u.sp_crd("AX")] = euler[0]
        self.basePoseW[self.u.sp_crd("AY")] = euler[1]
        self.basePoseW[self.u.sp_crd("AZ")] = euler[2]
        self.basePoseW[self.u.sp_crd("LX")] = msg.pose.pose.position.x
        self.basePoseW[self.u.sp_crd("LY")] = msg.pose.pose.position.y
        self.basePoseW[self.u.sp_crd("LZ")] = msg.pose.pose.position.z

        self.comPoseW[self.u.sp_crd("AX")] = euler[0]
        self.comPoseW[self.u.sp_crd("AY")] = euler[1]
        self.comPoseW[self.u.sp_crd("AZ")] = euler[2]


        self.b_R_w = self.mathJet.rpyToRot(euler[0], euler[1], euler[2])
        W_offCoM = self.b_R_w.transpose().dot(self.B_offCoM)
        self.comPoseW[self.u.sp_crd("LX")] = msg.pose.pose.position.x + W_offCoM[self.u.crd("X")]
        self.comPoseW[self.u.sp_crd("LY")] = msg.pose.pose.position.y + W_offCoM[self.u.crd("Y")]
        self.comPoseW[self.u.sp_crd("LZ")] = msg.pose.pose.position.z + W_offCoM[self.u.crd("Z")]


        # print ("com pose ", self.basePoseW)
        # print ("com offset ", self.B_offCoM)

        self.baseTwistW[self.u.sp_crd("AX")] = msg.twist.twist.angular.x
        self.baseTwistW[self.u.sp_crd("AY")] = msg.twist.twist.angular.y
        self.baseTwistW[self.u.sp_crd("AZ")] = msg.twist.twist.angular.z
        self.baseTwistW[self.u.sp_crd("LX")] = msg.twist.twist.linear.x
        self.baseTwistW[self.u.sp_crd("LY")] = msg.twist.twist.linear.y
        self.baseTwistW[self.u.sp_crd("LZ")] = msg.twist.twist.linear.z




        self.comTwistW = np.dot(motionVectorTransform(W_offCoM, np.eye(3)), self.baseTwistW)

        #update Pinocchio stuff (TODO)
        # gen_velocities = np.hstack((self.baseTwistW, self.qd))
        # configuration = np.hstack((self.u.linPart(self.basePoseW), self.quaternion, self.q))
        # self.h_joints = pin.nonLinearEffects(self.model, self.data, configuration, gen_velocities)

        #####################################
        # compare numerical differentiation
        ##################################
        # current_time = msg.header.stamp
        # try:
        #     p.dt_receive_pose = (current_time - self.last_time).to_sec()
        # except:
        #     pass
        # self.last_time = current_time
        # # check delays
        # # print("time stamp: ", msg.header.stamp.to_sec())
        # # print ("delay (ms): ", 1000.0*(ros.Time.now().to_sec() - msg.header.stamp.to_sec()))
        #
        # try:
        #     if (p.dt_receive_pose >= 1e-03):  # to discard messages that will arrive together
        #         p.comTwist_num = (p.comPoseW - p.comPoseWold) / p.dt_receive_pose
        # except:
        #     pass
        # # update
        # p.comPoseWold = copy.deepcopy(p.comPoseW)

    def _receive_jstate(self, msg):
        # need to map to robcogen only the arrays coming from gazebo because of ROS convention is different
        self.joint_names = msg.name
        q_ros = np.zeros(12)
        qd_ros = np.zeros(12)
        tau_ros = np.zeros(12)
        for i in range(len(self.joint_names)):
            q_ros[i] = msg.position[i]
            qd_ros[i] = msg.velocity[i]
            tau_ros[i] = msg.effort[i]
        # map to our convention
        self.q = self.u.mapFromRos(q_ros)
        self.qd = self.u.mapFromRos(qd_ros)
        self.tau = self.u.mapFromRos(tau_ros)

    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = JointState()
        msg.position = q_des
        msg.velocity = qd_des
        msg.effort = tau_ffwd
        self.pub_des_jstate.publish(msg)
        self.numberOfPublishedMessages += 1

    def send_centroidalState(self, des_state, des_forces, stance_legs):

        msg = self.msg_class()

        w_des_com_pos = self.u.getSegment(des_state, 0, 3)
        w_des_com_vel = self.u.getSegment(des_state, 3, 3)
        des_euler = self.u.getSegment(des_state, 6, 3)
        des_euler_rates = self.u.getSegment(des_state, 9, 3)

        msg.com_position = w_des_com_pos
        msg.com_velocity = w_des_com_vel
        msg.euler_angles = des_euler
        msg.euler_rates = des_euler_rates

        msg.des_forces_LF = self.u.getSegment(des_forces, 0, 3)
        msg.des_forces_RF = self.u.getSegment(des_forces, 3, 3)
        msg.des_forces_LH = self.u.getSegment(des_forces, 6, 3)
        msg.des_forces_RH = self.u.getSegment(des_forces, 9, 3)
        msg.stance_legs.LF = stance_legs[0]
        msg.stance_legs.RF = stance_legs[1]
        msg.stance_legs.LH = stance_legs[2]
        msg.stance_legs.RH = stance_legs[3]
        self.pub_des_centroidal_state.publish(msg)

    def register_nodes(self,optiParam):

        # this should be done only once, anonymous does not append any string to node name
        ros.init_node('sub_pub_node_python', anonymous=False)

        self.robot_name = ros.get_param('/robot_name')
        # subscribers
        # contact
        self.sub_contact = ros.Subscriber("/" + self.robot_name + "/contacts_state", ContactsState,
                                          callback=self._receive_contact, queue_size=1)
        # base pose
        self.sub_pose = ros.Subscriber("/" + self.robot_name + "/ground_truth", Odometry, callback=self._receive_pose,
                                       queue_size=5)
        # TODO using this has less delays
        # self.sub_pose = ros.Subscriber("/gazebo/model_states", ModelStates, callback=self._receive_pose_real_time,  queue_size=5)

        # joint states
        self.sub_jstate = ros.Subscriber("/" + self.robot_name + "/joint_states", JointState,
                                         callback=self._receive_jstate, queue_size=1)
        # publishers
        # impedance controller
        self.pub_des_jstate = ros.Publisher("/" + self.robot_name + "/command", JointState, queue_size=1)

        ros.wait_for_service("/" + self.robot_name + "/freeze_base")


        # services

        #new freeze base
        #self.set_state_client = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_physics_client = ros.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.get_physics_client = ros.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)

        # freezeBase
        self.freeze_base = ros.ServiceProxy("/" + self.robot_name + "/freeze_base", Empty)
        self.pause_physics_client = ros.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = ros.ServiceProxy('/gazebo/unpause_physics', Empty)

        if (self.controller_type == 1):
            try:
                from dwl_plan_controller.msg import centroidalState
            except:
                 print( "well, dwl_plan_controller.msg WASN'T defined, check branch python controller or set controller_type: 0")
            else:
                print("USING dwl_plan_controller")
                self.msg_class = centroidalState
                self.pub_des_centroidal_state = ros.Publisher("/" + self.robot_name + "/centroidal_command",
                                                          centroidalState, queue_size=1)
                self.client_trunk_cont_on = ros.ServiceProxy("/" + self.robot_name + "/trunk_cont_on_service", Empty)
                self.client_trunk_cont_off = ros.ServiceProxy("/" + self.robot_name + "/trunk_cont_off_service", Empty)

        # save vars into param server for other nodes
        self.u.putIntoParamServer(optiParam.configLoaded)
        self.u.putIntoGlobalParamServer("verbose", self.verbose)
        self.u.putIntoGlobalParamServer("mapper_on", self.mapper_on)

        #starting nodes
        #get list of nodes alive
        nodes = rosnode.get_node_names()
        # start reference gen node
        if "/mpc/reference_gen" in nodes:
            print("ROSNODE: re-starting ref gen")
            os.system("rosnode kill /mpc/reference_gen")
        else:
            print("ROSNODE: starting ref gen")
        package = 'reference_generator'
        executable = 'reference_generator'
        name = 'reference_gen'
        namespace = '/mpc'
        node = roslaunch.core.Node(package, executable, name, namespace, output="screen")
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        process = self.launch.launch(node)
        print("ROSNODE: reference generator alive: ",process.is_alive())

        #start mapper  node
        if optiParam.mapper_on:
            nodes = rosnode.get_node_names()
            if "/dls_mapper_node" in nodes:
                print("ROSNODE: re-starting dls_mapper node")
                os.system("rosnode kill /dls_mapper_node")
            else:
                print("ROSNODE: starting dls_mapper")

            rospack = rospkg.RosPack()
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            node_path = [rospack.get_path('dls_mapper') + "/launch/simulation.launch"]
            self.launch = roslaunch.parent.ROSLaunchParent(uuid, node_path)
            self.launch.start()


        print("Registered Nodes")

    def setSimSpeed(self, ts):
        physics_req = SetPhysicsPropertiesRequest()
        physics_req.time_step = ts
        physics_req.max_update_rate = 1000
        physics_req.ode_config.sor_pgs_iters = 50
        physics_req.ode_config.sor_pgs_w = 1.3
        physics_req.ode_config.contact_surface_layer = 0.001
        physics_req.ode_config.contact_max_correcting_vel = 100
        physics_req.ode_config.erp = 0.2
        physics_req.ode_config.max_contacts = 20
        physics_req.gravity.z = -9.81
        self.set_physics_client(physics_req)

    # def freeze_base(self, flag):
    #
    #     # TODO make this code independent from framework because it countinuosly sets the gravity mode to false at the beginning till you call fb! so it will override this
    #     # toggle gravity
    #     req_reset_gravity = SetPhysicsPropertiesRequest()
    #     # ode config
    #     req_reset_gravity.time_step = 0.001
    #     req_reset_gravity.max_update_rate = 1000
    #     req_reset_gravity.ode_config.sor_pgs_iters = 50
    #     req_reset_gravity.ode_config.sor_pgs_w = 1.3
    #     req_reset_gravity.ode_config.contact_surface_layer = 0.001
    #     req_reset_gravity.ode_config.contact_max_correcting_vel = 100
    #     req_reset_gravity.ode_config.erp = 0.2
    #     req_reset_gravity.ode_config.max_contacts = 20
    #
    #     if (flag):
    #         req_reset_gravity.gravity.z = 0.0
    #     else:
    #         req_reset_gravity.gravity.z = -9.81
    #     self.set_physics_client(req_reset_gravity)
    #
    #     # create the message
    #     req_reset_world = SetModelStateRequest()
    #     # create model state
    #     model_state = ModelState()
    #     model_state.model_name = "hyq"
    #     model_state.pose.position.x = 0.0
    #     model_state.pose.position.y = 0.0
    #     model_state.pose.position.z = 2.0
    #     model_state.pose.orientation.w = 1.0
    #     model_state.pose.orientation.x = 0.0
    #     model_state.pose.orientation.y = 0.0
    #     model_state.pose.orientation.z = 0.0
    #     req_reset_world.model_state = model_state
    #     # send request and get response (in this case none)
    #     self.set_state_client(req_reset_world)

    def deregister_nodes(self, p):
        print("deregistering nodes")
        ros.signal_shutdown("manual kill")

    def get_sim_time(self):
        return self.sim_time

    def get_contact(self):
        return self.contactsW

    def get_pose(self):
        return self.basePoseW

    def get_jstate(self):
        return self.q

    def initKinematics(self, kin):
        kin.init_homogeneous()
        kin.init_jacobians()

    def updateKinematics(self, kin):
        # q is continuously updated
        kin.update_homogeneous(self.q)
        kin.update_jacobians(self.q)
        self.actual_feetB = kin.forward_kin(self.q)
        # add 2cm to the last column
        self.actual_feetB += self.foot_ball_correction
        self.actual_feetW = kin.forward_kinW(self.q, self.b_R_w, self.u.linPart(self.get_pose()),
                                             self.foot_ball_correction)
        # update the feet jacobians
        self.J[self.u.leg_map("LF")], self.J[self.u.leg_map("RF")], self.J[self.u.leg_map("LH")], self.J[self.u.leg_map("RH")], flag = kin.getLegJacobians()
        #map jacobians to WF
        for leg in range(4):
            self.wJ[leg] = self.b_R_w.transpose().dot(self.J[leg])


    def unfoldResult(self, result, opt):
        # unfold the optimization result
        x_pred = np.zeros((opt.nx, opt.N + 1))
        u_opt = np.zeros((opt.nu, opt.N))
        k = 0

        for k in range(opt.N + 1):
            x_pred[:, k] = result.x[(k * opt.nx):(k * opt.nx) + opt.nx]
            if k <= opt.N - 1:
                u_opt[:, k] = result.x[(opt.nx * (opt.N + 1)) + (k * 12):(opt.nx * (opt.N + 1)) + (k * 12) + 12]
        # print (x_pred)
        # print (u_opt)
        return x_pred, u_opt

    def updateInitialState(self):
        self.initialCom = np.copy(self.u.linPart(self.comPoseW))
        self.initialOrient = np.copy(self.u.angPart(self.comPoseW))
        self.initialAngVel = np.copy(self.u.angPart(self.baseTwistW))
        self.initialLinVel = np.copy(self.u.linPart(self.baseTwistW))

    def getNewReference(self, optimizer, refclass, solver, ref_counter):
        # get initial state
        self.updateInitialState()
        # get reference

        refclass.getReferenceData(self.current_swing, self.swing_counter, self.robotHeight, self.initialCom,
                                  self.initialOrient, self.desLinVelocity, self.desHeadingVelocity, self.actual_feetW, self.delta_step, ref_counter)
        x_init = np.array([self.initialCom, self.initialLinVel, self.initialOrient, self.initialAngVel]).flatten()


        if self.verbose == 1:
            print("initial COM for ref", x_init)
            print("--------------------------------------------------------------")
        # to debug reference
        # plotReference(refclass)
        # linearization only needed for solvers other than acados
        if solver == 3:
            pass
        else:
            optimizer.linearize(refclass, x_init)
            if self.verbose == 1:
                print("lin_time: --- %s seconds ---" % optimizer.lin_time)

    def computeOptimization(self, optimizer, startIndex, endIndex, firstTime):
        self.updateInitialState()  # THIS HAS BEEN COMMENTED FOR NOW
        x_init = np.array([self.initialCom, self.initialLinVel, self.initialOrient, self.initialAngVel]).flatten()
        if (self.verbose) == 1:
            print("initial COM for Optimization", x_init)
        # MPC
        if (self.solver == 0):
            # solve the problem using the cvx. Note: this is symbolic and slow
            des_states, des_forces = optimizer.cvx_solve(self.initialCom, self.initialOrient, self.initialLinVel,
                                                 self.initialAngVel, startIndex, endIndex)
            des_force_dot = np.zeros((self.nu, self.prediction_horizon))
        if (self.solver == 1):
            optimizer.construct_standard_qp(self.initialCom, self.initialOrient, self.initialLinVel, self.initialAngVel,
                                          startIndex, endIndex)
            result = optimizer.gurobi_solve()
            des_states, des_forces = self.unfoldResult(result, optimizer)
            des_force_dot = np.zeros((self.nu, self.prediction_horizon))

        if (self.solver == 2):
            optimizer.construct_standard_qp(self.initialCom, self.initialOrient, self.initialLinVel, self.initialAngVel,
                                          startIndex, endIndex)
            # #perform optimization with osqp
            result = optimizer.osqp_solve()
            des_states, des_forces = self.unfoldResult(result, optimizer)
            des_force_dot = np.zeros((self.nu, self.prediction_horizon))

        if (self.solver == 3):
            # Pass initial condition, reference trajectory, linearization trajectory,
            # foot location and stance vector to the OCP object
            # Note: except initial condition all other parameters are already populate in 'optimizer' internally
            if firstTime:
                x_warmStart = np.zeros((12, self.prediction_horizon))
                u_warmStart = np.zeros((12, self.prediction_horizon))
            else:
                x_warmStart = np.hstack([self.x_forWarmStart[:, self.replanning_window - 1:],
                                         np.kron(self.x_forWarmStart[:, -1],
                                         np.ones((self.replanning_window-1, 1))).transpose()])
                u_warmStart = np.hstack([self.u_forWarmStart[:, self.replanning_window - 1:],
                                         np.kron(self.u_forWarmStart[:, -1],
                                         np.ones((self.replanning_window, 1))).transpose()])
            optimizer.pass_data_to_acados_ocp(self.initialCom, self.initialOrient,
                                         self.initialLinVel, self.initialAngVel,
                                         firstTime,
                                         x_warmStart, u_warmStart)
            ocp_solver = optimizer.ocp_solver

            # update linearization trajectory only at the first run of replanning
            # for consecutive run the linearization trajectory is used from memory
            # as per the warm start feature of SQP_RTI scheme
            # Note: This is valid when optimization is run at every iteration
            # For replanning at a certain window requires updating linearization trajctory
            # if firstTime:
            print("updating linearization trajectory")
            ocp_solver.update_lin_trajectory()

            # update initial condition, references, foot location and stance vector
            ocp_solver.update_ocp_solver_param()

            # if firstTime and ocp_solver.acados_solver.acados_ocp.solver_options.nlp_solver_type == 'SQP_RTI':
            if False:
                # run optimization
                iter = 3
                for i in range(iter):
                    print("--------------------------------------------------------------")
                    print("RTI iteration:", i)
                    print("--------------------------------------------------------------")
                    # print("acados solver type",ocp_solver.acados_solver.acados_ocp.solver_options.nlp_solver_type)
                    if i == iter - 1:
                        des_states, des_forces = ocp_solver.run_ocp_solver()
                    else:
                        ocp_solver.run_ocp_solver()
            else:
                # run optimization
                if self.model_type == 2:
                    des_states, des_forces, des_force_dot = ocp_solver.run_ocp_solver()
                    self.x_forWarmStart = des_states
                    self.u_forWarmStart = des_forces
                else:
                    # If CoM frame model transform omega to world frame
                    des_states, des_forces = ocp_solver.run_ocp_solver()
                    des_force_dot = np.zeros((self.nu, self.prediction_horizon))
                    self.x_forWarmStart = des_states
                    self.u_forWarmStart = des_forces

        # DEBUG - check cones
        # plotCones(p, des_forces, optimizer)
        if self.verbose == 1:
            print("opt_time: --- %s seconds ---" % optimizer.opt_time)

        return des_states, des_forces, des_force_dot

    def computeJointVariables(self, p, des_state, ref_feetW_horizon, counter_inside_reference_window):

        math = Math()
        des_com_pos = self.u.getSegment(des_state, 0, 3)
        des_orient = self.u.getSegment(des_state, 6, 3)
        b_R_w_des = math.rpyToRot(des_orient[self.u.crd("X")], des_orient[self.u.crd("Y")], des_orient[self.u.crd("Z")])

        Wdes_base = des_com_pos - b_R_w_des.transpose().dot(self.B_offCoM) # do not use actual does not work!

        self.ref_feetW = ref_feetW_horizon[:, counter_inside_reference_window]  # is a 12D vector
        w_foot_rel_ref = [0] * 4
        b_foot_rel_ref = [0] * 4
        for leg in range(4):
            # detect lift off
            lift_off_event = self.u.detectLiftOff(self.refclass.swing, counter_inside_reference_window, leg, p.prediction_horizon)
            if  self.stance_legs[leg] and lift_off_event:

                print("---------------------------start swinging leg ", self.u.leg_name(leg))
                self.pid.setPDleg(leg, 400.0, 6.0, 0.0)
                self.sampleFootW[leg] = self.des_feetW[leg] #before was self.u.getLegJointState(leg, self.ref_feetW)
                self.swing_time[leg] = 0.0
                self.computeStepLength(leg, ref_feetW_horizon)
                self.current_swing = leg
                self.swinging_legs[leg] = True
                self.stance_legs[leg] = False

            #  swing
            if (self.swinging_legs[leg]):
                swingPosW = np.array([0.0, 0.0, 0.0])
                swingPosSF = np.array([0.0, 0.0, 0.0])
                swingPosSF[self.u.crd("X")] = self.step_length[leg] / 2 * (1 - np.cos(np.pi * (self.swing_frequency * self.swing_time[leg])))
                swingPosSF[self.u.crd("Y")] = 0.0
                swingPosSF[self.u.crd("Z")] = self.step_height * np.sin(np.pi * (self.swing_frequency * self.swing_time[leg]))

                swingPosW = np.dot(self.w_R_sw, swingPosSF)
                self.des_feetW[leg] = self.sampleFootW[leg] + swingPosW


                # update time
                self.swing_time[leg] += self.control_resolution
                swing_down = self.swing_time[leg] >= (self.swing_duration / 2.0)

                # detect touchdown
                haptic_touch_down_event = self.u.detectHapticTouchDown(self.grForcesW, leg, self.force_th)
                non_haptic_touch_down_event = self.u.detectTouchDown(self.refclass.swing, counter_inside_reference_window, leg)

                if  swing_down and ((self.haptic_td and haptic_touch_down_event) or  (not self.haptic_td and non_haptic_touch_down_event)):
                    print("-------------------------touch down leg ",  self.u.leg_name(leg))
                    self.pid.setPDleg(leg, 50.0, 5.0, 0.0)
                    self.stance_legs[leg] = True
                    self.swinging_legs[leg] = False
                    self.swing_counter = 0

                    self.des_feetW[leg] = self.actual_feetW[leg]


            # compute relative feet trajectory in the world frame
            w_foot_rel_ref[leg] = self.des_feetW[leg]  - Wdes_base
            #act_base = self.u.linPart(self.basePoseW)
            #w_foot_rel_ref[leg] = self.des_feetW[leg] - act_base

            # map to base frame
            # the ikin is done considering the foot at the center of the ball
            b_foot_rel_ref[leg] = b_R_w_des.dot(w_foot_rel_ref[leg] - self.foot_ball_correction)
            #b_foot_rel_ref[leg] = self.b_R_w.dot(w_foot_rel_ref[leg] - self.foot_ball_correction)
            q_des_leg = self.kin.leg_inverse_kin(leg, b_foot_rel_ref[leg], np.array([0, 0, 0]))
            # print q_des_leg
            self.u.setLegJointState(leg, q_des_leg, self.q_des)
            self.qd_des = np.zeros(12)   # TODO use com offet, and compute qd_des appropriately according to the motion (now we set it to zero)

    def computeStepLength(self, leg, ref_feetW_horizon):
        #updates the step length and the rotation matrix w_R_sw

        next_feetW = ref_feetW_horizon[:, self.counter_inside_reference_window - self.swing_counter + np.int(self.swing_duration / self.time_resolution)]
        # compute step length
        self.delta_step[leg] = self.u.getLegJointState(leg, next_feetW) - self.sampleFootW[leg]
        self.step_length[leg] = np.linalg.norm(self.delta_step[leg])
        print("step length is ", self.step_length[leg])


        # check step length is not zero to avoid division by zero
        if (self.step_length[leg] > 0.0):
            w_swing_axis_x = self.delta_step[leg] / self.step_length[leg]
        else:
            print(colored("WARNING Step length is very small", "red"))
            w_swing_axis_x = np.array([1.0, 0.0, 0.0])
            self.step_length[leg] = 0.2
            self.delta_step[leg] = self.step_length[leg]*w_swing_axis_x

        world_axis_z = np.array([0.0, 0.0, 1.0])
        w_swing_axis_y = np.cross(world_axis_z, w_swing_axis_x)  # TODO check when is zero!
        w_swing_axis_z = np.cross(w_swing_axis_x, w_swing_axis_y)

        # work out w_R_sw = [w_swing_axis_x |  w_swing_axis_y | w_swing_axis_z]
        self.w_R_sw = np.eye(3)
        self.w_R_sw[:, self.u.crd("X")] = w_swing_axis_x
        self.w_R_sw[:, self.u.crd("Y")] = w_swing_axis_y
        self.w_R_sw[:, self.u.crd("Z")] = w_swing_axis_z


    def computeControl(self, p, des_state, suggested_forces):
        math = Math()
        des_com_pos = self.u.getSegment(des_state, 0, 3)
        des_orient = self.u.getSegment(des_state, 6, 3)
        b_R_w_des = math.rpyToRot(des_orient[self.u.crd("X")], des_orient[self.u.crd("Y")], des_orient[self.u.crd("Z")])
        Wdes_base = des_com_pos - b_R_w_des.transpose().dot(self.B_offCoM)

        # compute des_grf from whole-body controller
        B_contacts = self.kin.forward_kin(self.q) + self.foot_ball_correction

        # map contactct to wf
        w_R_b = self.b_R_w.transpose()
        W_contacts = np.zeros((3, 4))
        W_contacts[:, self.u.leg_map("LF")] = w_R_b.dot(B_contacts[self.u.leg_map("LF"), :].transpose()) + Wdes_base
        W_contacts[:, self.u.leg_map("RF")] = w_R_b.dot(B_contacts[self.u.leg_map("RF"), :].transpose()) + Wdes_base
        W_contacts[:, self.u.leg_map("LH")] = w_R_b.dot(B_contacts[self.u.leg_map("LH"), :].transpose()) + Wdes_base
        W_contacts[:, self.u.leg_map("RH")] = w_R_b.dot(B_contacts[self.u.leg_map("RH"), :].transpose()) + Wdes_base

        des_com_pose = np.hstack([self.u.getSegment(des_state, 0, 3), self.u.getSegment(des_state, 6, 3)])
        des_com_twist = np.hstack([self.u.getSegment(des_state, 3, 3), self.u.getSegment(des_state, 9, 3)])
        des_com_acc = np.subtract(des_com_twist, self.des_com_twist_old) / self.Ts
        self.des_com_twist_old = des_com_twist

        if (self.controller_type == 0):

            #with ros_impedance controller we compute our own whole body controller
            p.WBcontrForcesW = wholeBodyController(self.comPoseW, self.comTwistW, W_contacts, des_com_pose, des_com_twist,
                                                   np.zeros(6),
                                                   self.stance_legs)
            #################################
            # convert des grf forces into torques
            #################################
            # self weight
            self_weight = np.array(
                [-2.1342, 3.91633, -0.787648, -2.13605, 3.9162, -0.78766, -2.10752, -3.77803, 0.781712, -2.10583, -3.77811,
                 0.781689])
            p.jacsT = block_diag(np.transpose(p.wJ[p.u.leg_map("LF")]),
                            np.transpose(p.wJ[p.u.leg_map("RF")] ),
                            np.transpose(p.wJ[p.u.leg_map("LH")] ),
                            np.transpose(p.wJ[p.u.leg_map("RH")]  ))
            # /TODO
            # if (np.norm(getLegJointState(legId, p.WBcontrForcesW)) < 0.1) => leg is in swing and you set rows to zero in jacsT matrix relative to that leg
            # assemble the Jb matrix
            # do pseudoinverse of Jcb^T (Jcb_inv)
            # self.tau_ffwd =  -np.dot (np.dot(self.jacsT , Jcb_inv , WrenchBase)
            self.tau_ffwd = self_weight - self.jacsT.dot(p.WBcontrForcesW)
            self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)

        if (self.controller_type == 1):
            # do not send tau ffwd
            self.send_des_jstate(self.q_des, self.qd_des, np.zeros(12))
            self.send_centroidalState(des_state, suggested_forces, self.stance_legs)

        # p.ros_pub.add_arrow(W_contacts[:, p.u.leg_map["LF"]],   p.u.getLegJointState(p.u.leg_map["LF"], p.grForcesW / 400), "green")
        # p.ros_pub.add_arrow(W_contacts[:, p.u.leg_map["RF"]],   p.u.getLegJointState(p.u.leg_map["RF"], p.grForcesW / 400), "green")
        # p.ros_pub.add_arrow(W_contacts[:, p.u.leg_map["LH"]],   p.u.getLegJointState(p.u.leg_map["LH"], p.grForcesW / 400), "green")
        # p.ros_pub.add_arrow(W_contacts[:, p.u.leg_map["RH"]],   p.u.getLegJointState(p.u.leg_map["RH"], p.grForcesW / 400), "green")
        # # plot desired grfs
        # p.ros_pub.add_arrow(W_contacts[:, p.u.leg_map["LF"]],   p.u.getLegJointState(p.u.leg_map["LF"], p.WBcontrForcesW / 400), "blue")
        # p.ros_pub.add_arrow(W_contacts[:, p.u.leg_map["RF"]],   p.u.getLegJointState(p.u.leg_map["RF"], p.WBcontrForcesW / 400), "blue")
        # p.ros_pub.add_arrow(W_contacts[:, p.u.leg_map["LH"]],   p.u.getLegJointState(p.u.leg_map["LH"], p.WBcontrForcesW / 400), "blue")
        # p.ros_pub.add_arrow(W_contacts[:, p.u.leg_map["RH"]],   p.u.getLegJointState(p.u.leg_map["RH"], p.WBcontrForcesW / 400), "blue")
        # p.ros_pub.publishVisual()

    def startupProcedure(self,optiConfig):
        print("Startup procedure...")
        self.unpause_physics_client(EmptyRequest())  # pulls robot up
        ros.sleep(0.2)  # wait for callback to fill in jointmnames
        self.pid = PidManager(self.joint_names, self.controller_type) #instantiate here cause I need jointnames

        self.setSimSpeed(0.002)

        # set joint pdi gains
        self.pid.setPDs(400.0, 10.0, 0.0)
        start_t = ros.get_time()

        # switch off active trunk controller
        if (self.controller_type == 1):
            print("switching off trunk controller...")
            self.client_trunk_cont_off()
            start_t += 4.0
            print("freeze base...")
            self.freeze_base()
            print("reset posture...")
            # GOZERO Keep the fixed configuration for the joints at the start of simulation
            self.q_des = np.array([-0.2, 0.75, -1.5, -0.2, 0.75, -1.5, -0.2, -0.75, 1.5, -0.2, -0.75, 1.5])
            self.qd_des = np.zeros(12)
            self.tau_ffwd = np.zeros(12)
            while ros.get_time() - start_t < 1.0:
                self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
            if self.verbose == 1:
                print("q err prima freeze base", (self.q - self.q_des))
            print("unfreeze base...")
            self.freeze_base()
            if self.verbose == 1:
                print("q err pre grav comp", (self.q - self.q_des))
            start_t += 5.0
            print("compensating gravity...")
            # switch on trunk controller
            self.client_trunk_cont_on()
            while ros.get_time() - start_t < 1:
                ros.sleep(0.01)
            if self.verbose == 1:
                print("q err post grav comp", (self.q - self.q_des))
        else:
            # GOZERO Keep the fixed configuration for the joints at the start of simulation
            self.q_des = np.array([-0.2, 0.7, -1.4, -0.2, 0.7, -1.4, -0.2, -0.7, 1.4, -0.2, -0.7, 1.4])
            self.qd_des = np.zeros(12)
            self.tau_ffwd = np.zeros(12)
            gravity_comp = np.array(
                [24.2571, 1.92, 50.5, 24.2, 1.92, 50.5739, 21.3801, -2.08377, -44.9598, 21.3858, -2.08365, -44.9615])
            self.freeze_base()
            start_t = ros.get_time()
            print("reset posture...")
            while ros.get_time() - start_t < 1.5:
                self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
                ros.sleep(0.01)
            if self.verbose == 1:
                print("q err prima freeze base", (self.q - self.q_des))
            self.freeze_base()
            ros.sleep(2.0)
            if self.verbose == 1:
                print("q err pre grav comp", (self.q - self.q_des))
            print("compensating gravity...")
            start_t = ros.get_time()
            while ros.get_time() - start_t < 2.5:
                self.send_des_jstate(self.q_des, self.qd_des, gravity_comp)
                ros.sleep(0.01)
            if self.verbose == 1:
                print("q err post grav comp", (self.q - self.q_des))

        self.setSimSpeed(optiConfig.sim_speed_ts)
        print("startup accomplished...")

    def initVars(self, nu):
        self.swing_duration = (1 - self.duty_factor) * self.cycle_time
        self.swing_frequency = 1.0 / self.swing_duration
        # Init feedforward torques for the joints
        self.tau_ffwd_log = np.empty((self.nu, 0)) * np.nan
        self.basePoseW_log = np.empty((6, 0)) * np.nan
        self.comPoseW_log = np.empty((6, 0)) * np.nan
        self.baseTwistW_log = np.empty((6, 0)) * np.nan
        self.q_des_log = np.empty((self.nu, 0)) * np.nan
        self.q_log = np.empty((self.nu, 0)) * np.nan
        self.grForcesW_log = np.empty((self.nu, 0)) * np.nan
        self.WBcontrForcesW_log = np.empty((self.nu, 0)) * np.nan

        self.ref_state_log = np.empty((self.nx, 0)) * np.nan
        self.ref_force_log = np.empty((self.nu, 0)) * np.nan
        self.ref_feetW_log = np.empty((self.nu, 0)) * np.nan

        self.act_force_log = np.empty((self.nu, 0)) * np.nan
        self.act_state_log = np.empty((self.nx, 0)) * np.nan
        self.act_feetW_log = np.empty((self.nu, 0)) * np.nan

        self.des_force_log = np.empty((self.nu, 0)) * np.nan
        self.des_force_dot_log = np.empty((self.nu, 0)) * np.nan
        self.des_state_log = np.empty((self.nx, 0)) * np.nan
        self.des_feetW_log = np.empty((self.nu, 0))* np.nan

        self.stability_margin_log = np.array([])

        self.twist_num_log = np.empty((6, 0)) * np.nan
        self.twist_log = np.empty((6, 0)) * np.nan
        self.dt_receive_pose_log = []



        self.replanning_times = 0
        self.control_planning_freq_ratio = pymath.floor(self.time_resolution / self.control_resolution)
        self.swing_counter = 0
        self.current_swing = 0

        self.counter_control_loop = 0  # counter for the control loop
        self.counter_inside_reference_window = 0  # counter of 40ms intervals passed since the update of the reference
        self.startIndex = 0  # location of the optimization event INSIDE the reference
        self.counter_inside_replanned_window = 0  # counter of 40ms intervals passed since the beginning of replanning

        self.replanning_times = 0
        self.control_planning_freq_ratio = pymath.floor(self.time_resolution / self.control_resolution)
        self.swing_counter = 0
        self.current_swing = 0

        # init des vars for feet
        self.des_feetW = copy.deepcopy(self.actual_feetW)

        self.replanning_flag = True
        self.replanning_flag_log = np.array([])
        self.stance_flag_log = np.empty((4, 0))


    def logData(self, counter_inside_reference_window, des_state, des_forces, des_force_dot):


        # log the feedforward torques sent to the robot
        self.tau_ffwd_log = np.hstack((self.tau_ffwd_log, self.tau_ffwd.reshape(self.nu, -1)))
        self.basePoseW_log =  np.hstack((self.basePoseW_log, self.basePoseW.reshape(6, -1)))
        self.comPoseW_log =  np.hstack((self.comPoseW_log, self.comPoseW.reshape(6, -1)))
        self.baseTwistW_log = np.hstack((self.baseTwistW_log, self.baseTwistW_log.reshape(6, -1)))
        self.q_log  = np.hstack((self.q_log, self.q.reshape(self.nu,-1) ))
        self.q_des_log = np.hstack((self.q_des_log, self.q_des.reshape(self.nu,-1) ))
        self.grForcesW_log = np.hstack((self.grForcesW_log, self.grForcesW.reshape(self.nu,-1) ))
        self.WBcontrForcesW_log = np.hstack((self.WBcontrForcesW_log, self.WBcontrForcesW.reshape(self.nu,-1) ))

        self.ref_state_log = np.hstack((self.ref_state_log, self.ref_states[:, counter_inside_reference_window].reshape(self.nx, -1)))
        self.ref_force_log = np.hstack((self.ref_force_log, self.ref_forces[:, counter_inside_reference_window].reshape(self.nu, -1)))


        self.act_state_log = np.hstack((self.act_state_log, np.hstack((self.u.linPart(self.comPoseW),
                                                                     self.u.linPart(self.baseTwistW),
                                                                     self.u.angPart(self.comPoseW),
                                                                     self.u.angPart(self.baseTwistW))).reshape(self.nx, -1)))
        self.des_state_log = np.hstack((self.des_state_log, des_state.reshape(self.nu, -1)))


        self.act_force_log = np.hstack((self.act_force_log, self.grForcesW.reshape(self.nu, -1)))
        self.des_force_log = np.hstack((self.des_force_log, des_forces.reshape(self.nu, -1)))
        self.des_force_dot_log = np.hstack((self.des_force_dot_log,
                                            des_force_dot[:, counter_inside_reference_window].reshape(self.nu, -1)))

        #log feet variables
        des_12Dvector = np.zeros(12)
        act_12Dvector = np.zeros(12)
        for leg in range(4):
            self.u.setLegJointState(leg, self.des_feetW[leg].squeeze() , des_12Dvector)
            self.u.setLegJointState(leg, self.actual_feetW[leg] , act_12Dvector)
        self.ref_feetW_log = np.hstack((self.ref_feetW_log, self.ref_feetW.reshape(12, -1)))
        self.des_feetW_log = np.hstack((self.des_feetW_log, des_12Dvector.reshape(12, -1)))
        self.act_feetW_log = np.hstack((self.act_feetW_log, act_12Dvector.reshape(12, -1)))


        # compute stability margin
        comXY = self.u.linPart(self.comPoseW)
        comXY[self.u.crd("Z")] = 0.0
        stability_margin = self.u.margin_from_poly(comXY, self.stance_legs, self.actual_feetW)
        self.stability_margin_log = np.hstack((self.stability_margin_log, stability_margin))

        self.replanning_flag_log = np.hstack((self.replanning_flag_log, int(self.replanning_flag)))
        self.stance_flag_log = np.hstack((self.stance_flag_log, self.swinging_legs.reshape(4,-1)))

        try:
            self.twist_log = np.hstack((self.twist_log, self.comTwistW.reshape(6, -1)))
            self.twist_num_log = np.hstack((self.twist_num_log, self.comTwist_num.reshape(6, -1)))
            self.dt_receive_pose_log.append(self.dt_receive_pose)
        except:
            pass
