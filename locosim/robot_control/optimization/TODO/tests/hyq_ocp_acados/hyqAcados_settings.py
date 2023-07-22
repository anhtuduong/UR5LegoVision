# Setup OCP problem formulation for a quadruped
# Author: Niraj Rathod

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from centroidal_model import centroidal_model, centroidal_model_comFrame, \
                             centroidal_comFrameModel_DeltaU, \
                             coneConstraints,coneConstraints_DeltaU
from tools.mathutils import hipPositionW, hipZPositionW
import scipy.linalg
import numpy as np
from casadi import vertcat
from casadi import norm_2

def hyqAcados_settings(Tf, N, args):
    ''' Method for acados OCP formulation'''
    # -----------------------------------------------------------------------------
    # Export quadruped model
    if args['model_type'] == 0: # World frame model
        print("Using world frame model")
        print("--------------------------------------------------------------")
        model = centroidal_model(args['robotMass'], args['robotInertia'])
    elif args['model_type'] == 1:              # CoM frame model
        print("Using CoM frame model")
        print("--------------------------------------------------------------")
        model = centroidal_model_comFrame(args['robotMass'], args['robotInertia'])
    else:
        print("Using CoM frame delta_u model")
        print("--------------------------------------------------------------")
        model = centroidal_comFrameModel_DeltaU(args['nu'], args['robotMass'], args['robotInertia'])

    # Create render arguments
    ocp = AcadosOcp()
    ocp.model = model

    # Set dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu    # opti variable size
    ny_e = nx       # terminal opti variables

    # - set dimension for nlp
    ocp.dims.N = N     # - prediction horizon

    # - Flag if unilaterality is included
    if args['model_type'] == 2 and args['include_uniCons'] == 1 :
        print("including unilaterality constraints with delta_u model")
        print("--------------------------------------------------------------")
    elif args['model_type'] != 2 and args['include_uniCons'] == 1:
        print("including unilaterality constraints")
        print("--------------------------------------------------------------")

    # - Cone constraints dimensions
    if args['model_type'] == 2 and args['include_coneCons'] == 1:
        constraint = coneConstraints_DeltaU(nx, nu, args['ng'], args['mu'], args['friction_upperBound'])
        ocp.dims.ng = constraint.ng # no. polytopic constraints
        print("including cone constraints with delta_u model")
        print("--------------------------------------------------------------")
    elif args['include_coneCons'] == 1:
        # Import constraints
        constraint = coneConstraints(nx, args['ng'], args['mu'], args['friction_upperBound'])
        ocp.dims.ng = constraint.ng # no. polytopic constraints
        print("including cone constraints")
        print("--------------------------------------------------------------")

    #-----------------------------------------------------------------------------
    # Set state penalties
    Q = args['Q']

    # Set input penalties
    R = args['R']

    # Set terminal penalties
    Qe = args['Qe']

    # Mobility weights
    M = args['M']

    if args['include_mobility'] == 0:
        # Initialize OCP cost and type for acados
        # nlp_cost = ocp.cost
        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"

        # Define the cost
        ocp.cost.W = scipy.linalg.block_diag(Q, R)
        ocp.cost.W_e = Qe

        # Define the Linear LS cost
        # Track y for LLS s.t. y = Vx * (x-x_ref) + Vu * (u - u_ref) + Vz * (z - z_ref)
        # - state
        Vx = np.zeros((ny, nx))
        Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx = Vx

        # - input
        Vu = np.zeros((ny, nu))
        Vu[nx:, :nu] = np.eye(nu)
        ocp.cost.Vu = Vu

        # - terminal
        Vx_e = np.zeros((ny_e, nx))
        Vx_e[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx_e = Vx_e
    else:
    #==========================================================================
        print("including mobility in the cost")
        print("--------------------------------------------------------------")
        # ocp.cost = ocp.cost
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"

        # Define the cost
        ocp.cost.W = scipy.linalg.block_diag(Q, R, M)
        ocp.cost.W_e = Qe

        # Define nonlinear cost
        x = ocp.model.x
        u = ocp.model.u
        xf = ocp.model.p
        # xf = args.xf0
        comPosw = vertcat(x[0],x[1],x[2])

        # Euclidean distance of hip to foot position
        # [LF, RF, LH, RH ] = hipPositionW(comPosw,x[6],x[7],x[8])
        # hipToFposLF = norm_2(LF - xf[0:3])
        # hipToFposRF = norm_2(RF - xf[3:6])
        # hipToFposLH = norm_2(LH - xf[6:9])
        # hipToFposRH = norm_2(RH - xf[9:12])
        # ocp.model.cost_y_expr = vertcat(x, u, vertcat(hipToFposLF, hipToFposRF, hipToFposLH, hipToFposRH))

        # Z distance of hip to foot position
        [LF_z, RF_z, LH_z, RH_z] = hipZPositionW(comPosw, x[6], x[7], x[8])
        hipToFposLF = LF_z - xf[2]
        hipToFposRF = RF_z - xf[5]
        hipToFposLH = LH_z - xf[8]
        hipToFposRH = RH_z - xf[11]
        ocp.model.cost_y_expr = vertcat(x, u, vertcat(hipToFposLF, hipToFposRF, hipToFposLH, hipToFposRH))
        ocp.model.cost_y_expr_e = x

    #==========================================================================
    # Set initial references
    ocp.cost.yref = args['yref']
    ocp.cost.yref_e = args['yref_e']


    # =====================================================================
    # Trying to setup external cost but keep getting some error
    # ocp.cost.yref = np.zeros((ny, ))
    # ocp.cost.yref_e = np.zeros((ny_e, ))
    #
    # ocp.cost.cost_type = "EXTERNAL"
    # ocp.cost.cost_type_e = "EXTERNAL"
    #
    # x = ocp.model.x
    # u = ocp.model.u
    # ocp.model.cost_expr_ext_cost = vertcat(x, u).T @ ocp.cost.W @ vertcat(x, u)
    # # print((vertcat(x, u)-ocp.cost.yref).T @ ocp.cost.W @ (vertcat(x, u)-ocp.cost.yref))
    # ocp.model.cost_expr_ext_cost_e = x.T @ ocp.cost.W_e @ x
    # =====================================================================
    # -----------------------------------------------------------------------------
    # Setting OCP constraints

    # - state bounds
    # nlp_con.lbx = np.array(args['x_min'])
    # nlp_con.ubx = np.array(args['x_max'])
    # nlp_con.idxbx = np.array(list(range(nx)))

    # Initialize state
    # ocp.constraints.x0 = args['x0']
    if args['model_type'] == 2:
        ocp.constraints.idxbx_0 = np.arange(nx-nu)
    else:
        ocp.constraints.idxbx_0 = np.arange(nx)
    ocp.constraints.lbx_0 = args['x0']
    ocp.constraints.ubx_0 = args['x0']

    # Initialize parameters
    ocp.parameter_values = args['xf0']

    # - input bounds
    if args['model_type'] == 2 and args['include_uniCons'] == 1:
        ocp.constraints.idxbx = np.arange(nu+2, nx, 3)
        ocp.constraints.idxbx_0 = np.hstack([np.arange(12),np.arange(nu+2, nx, 3)])
        ocp.constraints.lbx = np.array(args['u_min'])
        ocp.constraints.ubx = np.array(args['u_max'])
        print(ocp.constraints.idxbx_0)
    elif args['model_type'] != 2 and args['include_uniCons'] == 1:
        ocp.constraints.lbu = np.array(args['u_min'])
        ocp.constraints.ubu = np.array(args['u_max'])
        ocp.constraints.idxbu = np.arange(2, 12, 3)
        if args['include_uniSlack'] and not args['include_coneNuniSlack']:
            print("adding slacks for unilaterality constraints")
            print("--------------------------------------------------------------")
            ocp.constraints.idxsbu = np.arange(4)
            ocp.cost.zl = 10.0 * np.ones(4)
            ocp.cost.Zl = 0.0 * np.ones(4)  # hessian
            ocp.cost.zu = 10.0 * np.ones(4)
            ocp.cost.Zu = 0.0 * np.ones(4)  # hessian

    # cone constraints (Polytopic constraints)
    if args['include_coneCons'] == 1:
        ocp.constraints.C = constraint.C
        ocp.constraints.D = constraint.D
        ocp.constraints.lg = constraint.lg
        ocp.constraints.ug = constraint.ug
        # print(ocp.constraints.C)
        if args['include_coneSlack'] and not args['include_coneNuniSlack']:
            print("adding slacks for cone constraints")
            print("--------------------------------------------------------------")
            ocp.constraints.idxsg = np.arange(constraint.ng)
            ocp.cost.zl = 10.0 * np.ones(constraint.ng)
            ocp.cost.Zl = 0.0 * np.ones(constraint.ng)  # hessian
            ocp.cost.zu = 10.0 * np.ones(constraint.ng)
            ocp.cost.Zu = 0.0 * np.ones(constraint.ng)  # hessian

    # Add slacks for both bounds and polytopic constraints
    # if args.include_coneNuniSlack and args.include_coneSlack and args.include_uniSlack:
    if args['include_coneNuniSlack'] :
        assert (args['include_coneCons']), "set include_coneCons = 1!"
        assert (args['include_uniCons']), "set include_uniCons = 1!"
        # assert (args['include_coneSlack']), "set include_coneSlack = 1!"
        # assert (args['include_uniSlack']), "set include_uniSlack = 1!"
        print("adding slacks for both bounds and cone constraints")
        print("--------------------------------------------------------------")
        ocp.constraints.idxsbu = np.arange(4)
        ocp.constraints.idxsg = np.arange(constraint.ng)
        ocp.cost.zl = 10.0 * np.ones(constraint.ng+4)
        ocp.cost.Zl = 0.0 * np.ones(constraint.ng+4)  # hessian
        ocp.cost.zu = 10.0 * np.ones(constraint.ng+4)
        ocp.cost.Zu = 0.0 * np.ones(constraint.ng+4)  # hessian

    # Set QP solver and integration
    ocp.solver_options.tf = Tf
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    # ocp.solver_options.hessian_approx = "EXACT"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 1
    ocp.solver_options.sim_method_num_steps = 1
    # ocp.solver_options.sim_method_newton_iter = 1
    # ocp.solver_options.print_level = 0
    # ocp.solver_options.qp_solver_cond_N = N
    # ocp.solver_options.sim_method_newton_iter = 10

    # - NLP solver settings
    # ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    # ocp.solver_options.nlp_solver_tol_stat = 1e-9
    # ocp.solver_options.nlp_solver_tol_eq = 1e-9
    # ocp.solver_options.nlp_solver_tol_ineq = 1e-9
    # ocp.solver_options.nlp_solver_tol_comp = 1e-9
    # ocp.solver_options.nlp_solver_max_iter=0
    # ocp.solver_options.nlp_solver_step_length=1e-20

    # - QP solver settings
    # ocp.solver_options.qp_solver = "FULL_CONDENSING_QPOASES"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    # ocp.solver_options.qp_solver_tol_stat = 1e-14
    # ocp.solver_options.qp_solver_tol_eq = 1e-10
    # ocp.solver_options.qp_solver_tol_ineq = 1e-10
    # ocp.solver_options.qp_solver_tol_comp = 1e-9
    # ocp.solver_options.nlp_solver_max_iter=0
    # ocp.solver_options.qp_solver_iter_max = 10

    # Create solver
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

    return model, acados_solver

