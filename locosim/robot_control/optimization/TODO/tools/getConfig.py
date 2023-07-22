import yaml
import os
from tools.utils import Utils
import scipy.sparse as sparse
import numpy as np

current_dir = os.path.dirname(os.path.realpath(__file__))

class getConfig:
    def __init__(self):
        util = Utils()
        #read optimization params
        with open(current_dir+"/../config/config.yaml", 'r') as stream:
            data_loaded = yaml.load(stream) # use it for python2.7
            #data_loaded = yaml.full_load(stream)
        self.configLoaded = data_loaded

        # dimensions
        self.nx = data_loaded['nx']
        self.nu = data_loaded['nu']
        self.ng = data_loaded['ng']
        self.nj = data_loaded['nj']

        # User defined velocities
        self.desired_velocity_x = data_loaded['desired_velocity_x']
        self.desired_velocity_y = data_loaded['desired_velocity_y']
        self.desHeadingVelocity = data_loaded['desired_velocity_heading']

        # robot and gait data
        self.robotMass = data_loaded['robotMass']
        self.robotHeight = data_loaded['robotHeight']
        self.robotInertia = data_loaded['robotInertia']
        self.cycle_time = data_loaded['cycle_time']
        self.gait_type = data_loaded['gait_type']
        self.duty_factor = data_loaded['duty_factor']
        self.offset_a = data_loaded['offset_a']
        self.offset_b = data_loaded['offset_b']
        self.offset_c = data_loaded['offset_c']
        self.offset_d = data_loaded['offset_d']

        # Pallet info
        self.pallet_location = data_loaded['pallet_location']
        self.pallet_height = data_loaded['pallet_height']

        # Haptic touchdown flag
        self.haptic_td = data_loaded['haptic_td']
        self.force_th = data_loaded['force_th'] # touchdown force

        # delta_u regulation flag for gurobi
        self.deltaUregulationOn = data_loaded['deltaUregulationOn']

        # Prediction horizon same for reference
        self.prediction_horizon = data_loaded['prediction_horizon']

        # replanning flag
        self.replanning_on = data_loaded['replanning']

        # replanning window
        self.replanning_window = data_loaded['replanning_window']

        # Sampling time for planner
        self.time_resolution = data_loaded['time_resolution']
        self.Ts = self.time_resolution

        # Control loop duration
        self.control_resolution = data_loaded['control_resolution']

        # 0 - ros_impedance_controller 1 - dwl_plan_controller
        self.controller_type = data_loaded['controller_type']

        #0 cvx (not working), 1 gurobi, 2 osqp(not working), 3 acados
        self.solver = data_loaded['solver']

        # Type of model
        self.model_type = data_loaded['model_type']

        # Mobility in the cost
        self.include_mobility = data_loaded['include_mobility']

        # Include unilaterality constraints
        self.include_uniCons = data_loaded['include_uniCons']

        # Include cone constraints
        self.include_coneCons = data_loaded['include_coneCons']

        # include cone regularization in the cost
        self.include_cone_regularization = data_loaded['include_cone_regularization']

        # LTV model for linerization required for all the solvers except acados
        self.include_ltv_model = data_loaded['include_ltv_model']

        # load from text file
        self.load_from_txt = data_loaded['load_from_txt']

        # Verbosity flag
        self.verbose = data_loaded['verbose']

        #mapper
        self.mapper_on = data_loaded['mapper_on']

        self.experiment_duration = data_loaded['experiment_duration']

        #ts for gazebo sim
        self.sim_speed_ts = data_loaded['sim_speed_ts']

        class Active_plots:
            pass

        self.active_plots = Active_plots()
        self.active_plots.grforces = data_loaded['active_plots']['grforces']
        self.active_plots.com = data_loaded['active_plots']['com']
        self.active_plots.orientation = data_loaded['active_plots']['orientation']
        self.active_plots.delta_u = data_loaded['active_plots']['delta_u']
        self.active_plots.stability_margin = data_loaded['active_plots']['stability_margin']
        self.active_plots.feet = data_loaded['active_plots']['feet']
        self.active_plots.joint = data_loaded['active_plots']['joint']

        # =======================================================================================
        # OPTIMIZATION SETTINGS
        # =======================================================================================

        # - Weights
        # weights on states
        self.q_array = np.array([data_loaded['weight_Q']['com_x'],
                                 data_loaded['weight_Q']['com_y'],
                                 data_loaded['weight_Q']['com_z'],
                                 data_loaded['weight_Q']['linvel_x'],
                                 data_loaded['weight_Q']['linvel_y'],
                                 data_loaded['weight_Q']['linvel_z'],
                                 data_loaded['weight_Q']['ang_x'],
                                 data_loaded['weight_Q']['ang_y'],
                                 data_loaded['weight_Q']['ang_z'],
                                 data_loaded['weight_Q']['angvel_x'],
                                 data_loaded['weight_Q']['angvel_y'],
                                 data_loaded['weight_Q']['angvel_z']])
        self.Q = sparse.diags(self.q_array)

        # terminal weight
        self.qn_array = self.q_array
        self.QN = sparse.diags(self.qn_array)

        # weights on inputs
        self.r_array = np.array([data_loaded['weight_R']['LF_x'],
                                 data_loaded['weight_R']['LF_y'],
                                 data_loaded['weight_R']['LF_z'],
                                 data_loaded['weight_R']['RF_x'],
                                 data_loaded['weight_R']['RF_y'],
                                 data_loaded['weight_R']['RF_z'],
                                 data_loaded['weight_R']['LH_x'],
                                 data_loaded['weight_R']['LH_y'],
                                 data_loaded['weight_R']['LH_z'],
                                 data_loaded['weight_R']['RH_x'],
                                 data_loaded['weight_R']['RH_y'],
                                 data_loaded['weight_R']['RH_z']])
        self.R = sparse.diags(self.r_array)

        # weight for delta_u regulation
        self.deltaU_array = np.array([data_loaded['weight_du']['LF_x'],
                                      data_loaded['weight_du']['LF_y'],
                                      data_loaded['weight_du']['LF_z'],
                                      data_loaded['weight_du']['RF_x'],
                                      data_loaded['weight_du']['RF_y'],
                                      data_loaded['weight_du']['RF_z'],
                                      data_loaded['weight_du']['LH_x'],
                                      data_loaded['weight_du']['LH_y'],
                                      data_loaded['weight_du']['LH_z'],
                                      data_loaded['weight_du']['RH_x'],
                                      data_loaded['weight_du']['RH_y'],
                                      data_loaded['weight_du']['RH_z']])
        self.Lambda = sparse.diags(self.deltaU_array)

        # weights on mobility
        self.m_array = np.array([data_loaded['weight_M']['LF_x'],
                                 data_loaded['weight_M']['LF_y'],
                                 data_loaded['weight_M']['LF_z'],
                                 data_loaded['weight_M']['RF_x'],
                                 data_loaded['weight_M']['RF_y'],
                                 data_loaded['weight_M']['RF_z'],
                                 data_loaded['weight_M']['LH_x'],
                                 data_loaded['weight_M']['LH_y'],
                                 data_loaded['weight_M']['LH_z'],
                                 data_loaded['weight_M']['RH_x'],
                                 data_loaded['weight_M']['RH_y'],
                                 data_loaded['weight_M']['RH_z']])
        self.M = sparse.diags(self.m_array)

        # Regularization parameter
        self.reg_param = data_loaded['reg_param']

        # weights on cone regularization cost
        self.s_array = np.array([data_loaded['weight_S']['LF_tan_x'],
                                 data_loaded['weight_S']['LF_tan_y'],
                                 data_loaded['weight_S']['LF_normal'],
                                 data_loaded['weight_S']['RF_tan_x'],
                                 data_loaded['weight_S']['RF_tan_y'],
                                 data_loaded['weight_S']['RF_normal'],
                                 data_loaded['weight_S']['LH_tan_x'],
                                 data_loaded['weight_S']['LH_tan_y'],
                                 data_loaded['weight_S']['LH_normal'],
                                 data_loaded['weight_S']['RH_tan_x'],
                                 data_loaded['weight_S']['RH_tan_y'],
                                 data_loaded['weight_S']['RH_normal']])
        self.S = sparse.diags(self.s_array)

        # Centroidal + Kino dynamic model
        # weights on states
        self.q_array_kino = np.array([data_loaded['weight_Q_kino']['com_x'],
                                      data_loaded['weight_Q_kino']['com_y'],
                                      data_loaded['weight_Q_kino']['com_z'],
                                      data_loaded['weight_Q_kino']['linvel_x'],
                                      data_loaded['weight_Q_kino']['linvel_y'],
                                      data_loaded['weight_Q_kino']['linvel_z'],
                                      data_loaded['weight_Q_kino']['ang_x'],
                                      data_loaded['weight_Q_kino']['ang_y'],
                                      data_loaded['weight_Q_kino']['ang_z'],
                                      data_loaded['weight_Q_kino']['angvel_x'],
                                      data_loaded['weight_Q_kino']['angvel_y'],
                                      data_loaded['weight_Q_kino']['angvel_z'],
                                      data_loaded['weight_Q_kino']['q_HAA_LF'],
                                      data_loaded['weight_Q_kino']['q_HFE_LF'],
                                      data_loaded['weight_Q_kino']['q_KFE_LF'],
                                      data_loaded['weight_Q_kino']['q_HAA_RF'],
                                      data_loaded['weight_Q_kino']['q_HFE_RF'],
                                      data_loaded['weight_Q_kino']['q_KFE_RF'],
                                      data_loaded['weight_Q_kino']['q_HAA_LH'],
                                      data_loaded['weight_Q_kino']['q_HFE_LH'],
                                      data_loaded['weight_Q_kino']['q_KFE_LH'],
                                      data_loaded['weight_Q_kino']['q_HAA_RH'],
                                      data_loaded['weight_Q_kino']['q_HFE_RH'],
                                      data_loaded['weight_Q_kino']['q_KFE_RH']])

        self.Q_kino = sparse.diags(self.q_array_kino)

        # weights on inputs
        self.r_array_kino = np.array([data_loaded['weight_R_kino']['LF_x'],
                                      data_loaded['weight_R_kino']['LF_y'],
                                      data_loaded['weight_R_kino']['LF_z'],
                                      data_loaded['weight_R_kino']['RF_x'],
                                      data_loaded['weight_R_kino']['RF_y'],
                                      data_loaded['weight_R_kino']['RF_z'],
                                      data_loaded['weight_R_kino']['LH_x'],
                                      data_loaded['weight_R_kino']['LH_y'],
                                      data_loaded['weight_R_kino']['LH_z'],
                                      data_loaded['weight_R_kino']['RH_x'],
                                      data_loaded['weight_R_kino']['RH_y'],
                                      data_loaded['weight_R_kino']['RH_z'],
                                      data_loaded['weight_R_kino']['v_HAA_LF'],
                                      data_loaded['weight_R_kino']['v_HFE_LF'],
                                      data_loaded['weight_R_kino']['v_KFE_LF'],
                                      data_loaded['weight_R_kino']['v_HAA_RF'],
                                      data_loaded['weight_R_kino']['v_HFE_RF'],
                                      data_loaded['weight_R_kino']['v_KFE_RF'],
                                      data_loaded['weight_R_kino']['v_HAA_LH'],
                                      data_loaded['weight_R_kino']['v_HFE_LH'],
                                      data_loaded['weight_R_kino']['v_KFE_LH'],
                                      data_loaded['weight_R_kino']['v_HAA_RH'],
                                      data_loaded['weight_R_kino']['v_HFE_RH'],
                                      data_loaded['weight_R_kino']['v_KFE_RH']])
        self.R_kino = sparse.diags(self.r_array_kino)

        # terminal weight
        self.qn_array_kino = self.q_array_kino
        self.QN_kino = sparse.diags(self.qn_array_kino)

        # Foot position cost
        self.fp_array_kino = np.array([data_loaded['weight_fPos_kino']['LF_x'],
                                       data_loaded['weight_fPos_kino']['LF_y'],
                                       data_loaded['weight_fPos_kino']['LF_z'],
                                       data_loaded['weight_fPos_kino']['RF_x'],
                                       data_loaded['weight_fPos_kino']['RF_y'],
                                       data_loaded['weight_fPos_kino']['RF_z'],
                                       data_loaded['weight_fPos_kino']['LH_x'],
                                       data_loaded['weight_fPos_kino']['LH_y'],
                                       data_loaded['weight_fPos_kino']['LH_z'],
                                       data_loaded['weight_fPos_kino']['RH_x'],
                                       data_loaded['weight_fPos_kino']['RH_y'],
                                       data_loaded['weight_fPos_kino']['RH_z']])
        self.FP_kino = sparse.diags(self.fp_array_kino)

        # - State bounds
        self.x_min = np.array(np.zeros(self.nx))
        self.x_min[0] = data_loaded['x_limits']['com_x']['min']
        self.x_min[1] = data_loaded['x_limits']['com_y']['min']
        self.x_min[2] = data_loaded['x_limits']['com_z']['min']

        self.x_min[3] = data_loaded['x_limits']['linvel_x']['min']
        self.x_min[4] = data_loaded['x_limits']['linvel_y']['min']
        self.x_min[5] = data_loaded['x_limits']['linvel_z']['min']

        self.x_min[6] = data_loaded['x_limits']['ang_x']['min']
        self.x_min[7] = data_loaded['x_limits']['ang_y']['min']
        self.x_min[8] = data_loaded['x_limits']['ang_z']['min']

        self.x_min[9] = data_loaded['x_limits']['angvel_x']['min']
        self.x_min[10] = data_loaded['x_limits']['angvel_y']['min']
        self.x_min[11] = data_loaded['x_limits']['angvel_z']['min']

        self.x_max = np.array(np.zeros(self.nx))
        self.x_max[0] = data_loaded['x_limits']['com_x']['max']
        self.x_max[1] = data_loaded['x_limits']['com_y']['max']
        self.x_max[2] = data_loaded['x_limits']['com_z']['max']

        self.x_max[3] = data_loaded['x_limits']['linvel_x']['max']
        self.x_max[4] = data_loaded['x_limits']['linvel_y']['max']
        self.x_max[5] = data_loaded['x_limits']['linvel_z']['max']

        self.x_max[6] = data_loaded['x_limits']['ang_x']['max']
        self.x_max[7] = data_loaded['x_limits']['ang_y']['max']
        self.x_max[8] = data_loaded['x_limits']['ang_z']['max']

        self.x_max[9] = data_loaded['x_limits']['angvel_x']['max']
        self.x_max[10] = data_loaded['x_limits']['angvel_y']['max']
        self.x_max[11] = data_loaded['x_limits']['angvel_z']['max']

        # - Input bounds
        self.z_force_max = np.array(np.zeros(4))
        self.z_force_max[util.leg_map("LF")] = data_loaded['z_force_limits']['LF']['max']
        self.z_force_max[util.leg_map("RF")] = data_loaded['z_force_limits']['RF']['max']
        self.z_force_max[util.leg_map("LH")] = data_loaded['z_force_limits']['LH']['max']
        self.z_force_max[util.leg_map("RH")] = data_loaded['z_force_limits']['RH']['max']

        self.z_force_min = np.array(np.zeros(4))
        self.z_force_min[util.leg_map("LF")] = data_loaded['z_force_limits']['LF']['min']
        self.z_force_min[util.leg_map("RF")] = data_loaded['z_force_limits']['RF']['min']
        self.z_force_min[util.leg_map("LH")] = data_loaded['z_force_limits']['LH']['min']
        self.z_force_min[util.leg_map("RH")] = data_loaded['z_force_limits']['RH']['min']

        self.mu = np.array(np.zeros(4))
        self.mu[util.leg_map("LF")] = data_loaded['friction_coeff']['LF']
        self.mu[util.leg_map("RF")] = data_loaded['friction_coeff']['RF']
        self.mu[util.leg_map("LH")] = data_loaded['friction_coeff']['LH']
        self.mu[util.leg_map("RH")] = data_loaded['friction_coeff']['RH']

        self.friction_upperBound = np.array(np.zeros(4))
        self.friction_upperBound[util.leg_map("LF")] = data_loaded['friction_upperBound']['LF']
        self.friction_upperBound[util.leg_map("RF")] = data_loaded['friction_upperBound']['RF']
        self.friction_upperBound[util.leg_map("LH")] = data_loaded['friction_upperBound']['LH']
        self.friction_upperBound[util.leg_map("RH")] = data_loaded['friction_upperBound']['RH']

        # Joint Angle bounds
        self.joint_pos_min = np.array([data_loaded['joint_angle_limits']['LF_HAA']['min'],
                                       data_loaded['joint_angle_limits']['LF_HFE']['min'],
                                       data_loaded['joint_angle_limits']['LF_KFE']['min'],
                                       data_loaded['joint_angle_limits']['RF_HAA']['min'],
                                       data_loaded['joint_angle_limits']['RF_HFE']['min'],
                                       data_loaded['joint_angle_limits']['RF_KFE']['min'],
                                       data_loaded['joint_angle_limits']['LH_HAA']['min'],
                                       data_loaded['joint_angle_limits']['LH_HFE']['min'],
                                       data_loaded['joint_angle_limits']['LH_KFE']['min'],
                                       data_loaded['joint_angle_limits']['RH_HAA']['min'],
                                       data_loaded['joint_angle_limits']['RH_HFE']['min'],
                                       data_loaded['joint_angle_limits']['RH_KFE']['min']])
        self.joint_pos_max = np.array([data_loaded['joint_angle_limits']['LF_HAA']['max'],
                                       data_loaded['joint_angle_limits']['LF_HFE']['max'],
                                       data_loaded['joint_angle_limits']['LF_KFE']['max'],
                                       data_loaded['joint_angle_limits']['RF_HAA']['max'],
                                       data_loaded['joint_angle_limits']['RF_HFE']['max'],
                                       data_loaded['joint_angle_limits']['RF_KFE']['max'],
                                       data_loaded['joint_angle_limits']['LH_HAA']['max'],
                                       data_loaded['joint_angle_limits']['LH_HFE']['max'],
                                       data_loaded['joint_angle_limits']['LH_KFE']['max'],
                                       data_loaded['joint_angle_limits']['RH_HAA']['max'],
                                       data_loaded['joint_angle_limits']['RH_HFE']['max'],
                                       data_loaded['joint_angle_limits']['RH_KFE']['max']])

        # Load hip to foot position reference
        self.ref_hiptofoot_pos = np.array(
            [data_loaded['ref_hiptofoot_pos']['LF_x'], data_loaded['ref_hiptofoot_pos']['LF_y'], data_loaded['ref_hiptofoot_pos']['LF_z'],
             data_loaded['ref_hiptofoot_pos']['RF_x'], data_loaded['ref_hiptofoot_pos']['RF_y'], data_loaded['ref_hiptofoot_pos']['RF_z'],
             data_loaded['ref_hiptofoot_pos']['LH_x'], data_loaded['ref_hiptofoot_pos']['LH_y'], data_loaded['ref_hiptofoot_pos']['LH_z'],
             data_loaded['ref_hiptofoot_pos']['RH_x'], data_loaded['ref_hiptofoot_pos']['RH_y'], data_loaded['ref_hiptofoot_pos']['RH_z']])

        # =======================================================================================
        # Record the model type for the acados generated code in .yaml file
        model_type_dict = {'acados_model_type': self.model_type,
                           'acados_include_mobility': self.include_mobility,
                           'acados_include_uniCons':self.include_uniCons,
                           'acados_include_coneCons':self.include_coneCons,
                           'include_cone_regularization':self.include_cone_regularization}

        with open(current_dir+'/../config/model.yaml', 'w') as file:
            documents = yaml.dump(model_type_dict, file, allow_unicode=True, default_flow_style=False)

        # =======================================================================================
        # Check for the errors
        comparision = self.q_array == self.qn_array
        assert comparision.all(), "weight_Q and weight_QN should be same!!"

        # Throw an error while checking mobility on height motion (z)
        if self.include_mobility and not(self.q_array[2] == 0.0) and np.all(self.m_array[2:12:3]):
            raise ValueError('Weight on com_z should be 0.0 when mobility Z is ON')

        if not(self.include_mobility) and self.q_array[2] == 0.0:
            raise ValueError('Weight on com_z should be nonzero when mobility is OFF')

        # Throw an error when cone and uni constraints are ON and u_min is 0.0
        nnzero_umin=[]
        for leg in range(4):
            if self.z_force_min[leg] == 0.0:
                nnzero_umin = util.leg_name(leg)
                break
        if self.include_coneCons and self.include_uniCons and not (not nnzero_umin):
            raise ValueError('Cannot turn ON unilaterality when cone constraint is ON and ' +
                             nnzero_umin + ' u_min = 0.0')


        if not(self.include_coneCons) and self.include_cone_regularization:
            raise ValueError('Turn ON cone constraints to include cone regularization cost!!')



        # Throw an error while checking mobility on angular motion (xy)
        # if self.include_mobility and not(self.q_array[8] == 0.0) and np.all(self.m_array[0:12:3]) and np.all(self.m_array[1:12:3]):
        #     raise ValueError('Weight on ang_z should be 0.0 when mobility is ON')
        #
        # if not(self.include_mobility) and self.q_array[8] == 0.0:
        #     raise ValueError('Weight on ang_z should be nonzero when mobility is OFF')


        # print "Q", self.Q
        # print "QN", self.QN
        # print "R", self.R
        # print "umax", self.u_max
        # print "umin", self.u_min
        # print "friction_coeff" , self.mu
        # print "prediction_horizon", self.N
        # print "robotMass", self.robotMass
        # print "robotInertia", self.robotInertia
        # print "time_resolution", self.Ts
