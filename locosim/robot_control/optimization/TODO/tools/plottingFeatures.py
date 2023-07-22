
import matplotlib.pyplot as plt
import numpy as np
plt.ion()


def plotVelocity(p):
    plt.figure(2)
    plt.rcParams['axes.grid'] = True
    plt.subplot(3, 1, 1)
    plt.plot(p.twist_log[0, :], label="Gazebo", linestyle='--')
    plt.plot(p.twist_num_log[0, :], label="Numerical")
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
               ncol=2, mode="expand", borderaxespad=0.)
    plt.ylabel("com Xd", fontsize=10)

    plt.subplot(3, 1, 2)
    plt.plot(p.twist_log[1, :], label="Gazebo", linestyle='--')
    plt.plot(p.twist_num_log[1, :], label="Numerical")
    plt.ylabel("com Yd", fontsize=10)

    plt.subplot(3, 1, 3)
    plt.plot(p.dt_receive_pose_log, label="", linestyle='--')
    plt.ylabel("dt (ms)", fontsize=10)


def plotResult(p, active_plots):

    des_forces = p.des_force_log[:, :-1]
    des_states = p.des_state_log
    des_feet  =  p.des_feetW_log
    ref_forces = p.ref_force_log[:, :-1]
    ref_states = p.ref_state_log
    ref_feet = p.ref_feetW_log
    act_forces = p.act_force_log[:, :-1]
    act_states = p.act_state_log
    act_feet = p.act_feetW_log
    delta_u = p.des_force_dot_log[:, :-1]
    legend_desired = p.legend_desired
    legend_actual = p.legend_actual
    legend_ref = p.legend_ref
    stability_margin = p.stability_margin_log
    model_type = p.model_type

    plt.rcParams['axes.grid'] = True
    plt.close('all')
    lw_des = 2
    lw_act = 2

    if active_plots.grforces:
        # neet to transpose the matrix other wise it cannot be plot with numpy array
        fig = plt.figure()
        fig.suptitle("Ground reaction forces", fontsize=20)
        labels = ["LF x", "LF y", "LF z", "RF x", "RF y", "RF z", "LH x", "LH y", "LH z", "RH x", "RH y", "RH z"]
        idx_vector = [1, 3, 5, 2, 4, 6, 7, 9, 11, 8, 10, 12]

        for jidx in range(12):
            plt.subplot(6, 2, idx_vector[jidx])
            plt.ylabel(labels[jidx])
            plt.plot(des_forces[jidx, :], linestyle='-', color='red', label=legend_desired)
            plt.plot(act_forces[jidx, :], linestyle='-.', color='blue', label=legend_actual)
            plt.plot(ref_forces[jidx, :], linestyle='--', color='green', label=legend_ref)
            plt.grid(True)

            if (jidx ==1 or jidx ==4):
                # plt.legend()
                plt.legend(bbox_to_anchor=(0., 2.4, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

    if active_plots.com:

        fig = plt.figure()
        fig.suptitle("Com", fontsize=20)
        labels = ["com X", "com Y", "com Z"]
        idx_subplots = [1, 3, 5]
        idx_states = [0,1,2]


        for idx in range(3):
            plt.subplot(3, 2, idx_subplots[idx])
            plt.ylabel(labels[idx])
            plt.plot(des_states[idx_states[idx], :], linestyle='-', lw=lw_des, color='red', label=legend_desired)
            plt.plot(act_states[idx_states[idx], :], linestyle='-.', lw=lw_act, color='blue', label=legend_actual)
            plt.plot(ref_states[idx_states[idx], :], linestyle='--', lw=lw_act, color='green', label=legend_ref)
            plt.grid(True)

            if (idx == 0):
                # plt.legend()
                plt.legend(bbox_to_anchor=(0., 1.06, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

        labels =[ "com vel X", "com vel Y", "com vel Z"]
        idx_subplots = [2, 4, 6]
        idx_states = [3,4,5]

        for idx in range(3):
            plt.subplot(3, 2, idx_subplots[idx])
            plt.ylabel(labels[idx])
            plt.plot(des_states[idx_states[idx], :], linestyle='-', lw=lw_des, color='red', label=legend_desired)
            plt.plot(act_states[idx_states[idx], :], linestyle='-.', lw=lw_act, color='blue', label=legend_actual)
            plt.plot(ref_states[idx_states[idx], :], linestyle='--', lw=lw_act, color='green', label=legend_ref)
            plt.grid(True)

            if (idx == 0):
                plt.legend(bbox_to_anchor=(0., 1.06, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

    if active_plots.orientation:

        fig = plt.figure()
        fig.suptitle("Trunk Orientation", fontsize=20)
        labels = ["roll", "pitch", "yaw" ]
        idx_subplots = [1, 3, 5]
        idx_states = [6, 7, 8]

        for idx in range(3):
            plt.subplot(3, 2, idx_subplots[idx])
            plt.ylabel(labels[idx])
            plt.plot(des_states[idx_states[idx], :], linestyle='-', lw=lw_des, color='red', label=legend_desired)
            plt.plot(act_states[idx_states[idx],:], linestyle='-.', lw=lw_act, color='blue', label=legend_actual)
            plt.plot(ref_states[idx_states[idx], :], linestyle='--', lw=lw_act, color='green', label=legend_ref)
            plt.grid(True)

            if (idx == 0):
                # plt.legend()
                plt.legend(bbox_to_anchor=(0., 1.06, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

        labels = ["omega X", "omega Y", "omega Z"]
        idx_subplots = [2, 4, 6]
        idx_states = [9, 10, 11]

        for idx in range(3):
            plt.subplot(3, 2, idx_subplots[idx])
            plt.ylabel(labels[idx])
            plt.plot(des_states[idx_states[idx], :], linestyle='-', lw=lw_des, color='red', label=legend_desired)
            plt.plot(act_states[idx_states[idx],:], linestyle='-.', lw=lw_act, color='blue', label=legend_actual)
            plt.plot(ref_states[idx_states[idx], :], linestyle='--', lw=lw_act, color='green', label=legend_ref)
            plt.grid(True)

            if (idx == 0):
                plt.legend(bbox_to_anchor=(0., 1.06, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)



    # Input regulation [delta_u] plots
    if active_plots.delta_u:
        if model_type == 2:
            fig = plt.figure()
            fig.suptitle("Inputs regulation du/dt", fontsize=20)
            labels = ["LF x", "LF y", "LF z", "RF x", "RF y", "RF z", "LH x", "LH y", "LH z",
                      "RH x",
                      "RH y", "RH z"]
            idx_vector = [1, 3, 5, 2, 4, 6, 7, 9, 11, 8, 10, 12]

            for jidx in range(12):
                plt.subplot(6, 2, idx_vector[jidx])
                plt.ylabel(labels[jidx])
                plt.plot(delta_u[jidx, :], linestyle='-', color='red',
                         label=legend_desired)
                if (jidx == 1 or jidx == 4):
                    # plt.legend()
                    plt.legend(bbox_to_anchor=(0., 2.4, 1., .102), loc=5,
                               ncol=3, mode="expand", borderaxespad=0.)

    if active_plots.stability_margin:
        plt.figure()
        plt.suptitle("Stability Margin")
        plt.plot(stability_margin)


    if active_plots.feet:
        fig = plt.figure()
        fig.suptitle("Feet Positions", fontsize=20)
        labels = ["LF x", "LF y", "LF z", "RF x", "RF y", "RF z", "LH x", "LH y", "LH z", "RH x", "RH y", "RH z", "Replanning", "Replanning"]
        idx_vector = [1, 3, 5, 2, 4, 6, 7, 9, 11, 8, 10, 12]

        for jidx in range(12):
            plt.subplot(7, 2, idx_vector[jidx])
            plt.ylabel(labels[jidx])
            plt.plot(des_feet[jidx, :].T, linestyle='-', lw=lw_des, color='red', label=legend_desired)
            plt.plot(act_feet[jidx, :].T, linestyle='-.', lw=lw_act, color='blue', label=legend_actual)
            plt.plot(ref_feet[jidx, :].T, linestyle='--', lw=lw_act, color='green', label=legend_ref)
            plt.grid(True)

            if (jidx == 1 or jidx == 4):
                # plt.legend()
                plt.legend(bbox_to_anchor=(0., 2.4, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

        #plot replanning events
        plt.subplot(7, 2, 13)
        plt.plot(p.replanning_flag_log, linestyle='-', lw=lw_act, color='black', label='Replanning')
        plt.plot(p.stance_flag_log[0,:], linestyle='-.', lw=lw_act, color='red', label='stance LF')
        plt.plot(p.stance_flag_log[2,:], linestyle='--', lw=lw_act, color='blue', label='stance LH')
        # plt.legend()
        plt.legend(bbox_to_anchor=(0., -0.4, 1., .102), loc=5,
                   ncol=3, mode="expand", borderaxespad=0.)
        plt.subplot(7, 2, 14)
        plt.plot(p.replanning_flag_log, linestyle='-', lw=lw_act, color='black', label='Replanning')
        plt.plot(p.stance_flag_log[1,:], linestyle='-.', lw=lw_act, color='green', label='stance RF')
        plt.plot(p.stance_flag_log[3,:], linestyle='--', lw=lw_act, color='gray', label='stance RH')
        # plt.legend()
        plt.legend(bbox_to_anchor=(0., -0.4, 1., .102), loc=5,
                   ncol=3, mode="expand", borderaxespad=0.)
    plt.show(block=True)

def plotTorques(p):
    # Torques

    plt.figure()
    plt.title("Torques")
    plt.subplot(6, 2, 1)
    plt.plot(p.tau_ffwd_log[0, :], label="Optimal")
    # plt.plot(ref_forces[0, :],  label="Ref", color='b', linestyle='--')
    plt.ylabel(r"$(tau_{LF})_x$", fontsize=10)

    plt.subplot(6, 2, 3)
    plt.plot(p.tau_ffwd_log[1, :])
    # plt.plot(ref_forces[1, :], linestyle='--')
    plt.ylabel(r"$(tau_{LF})_y$", fontsize=10)

    plt.subplot(6, 2, 5)
    plt.plot(p.tau_ffwd_log[2, :])
    # plt.plot(ref_forces[2, :], linestyle='--')
    plt.ylabel(r"$(tau_{LF})_z$", fontsize=10)

    plt.subplot(6, 2, 2)
    plt.plot(p.tau_ffwd_log[3, :])
    # plt.plot(ref_forces[3, :], linestyle='--')
    plt.ylabel(r"$(tau_{RF})_x$", fontsize=10)

    plt.subplot(6, 2, 4)
    plt.plot(p.tau_ffwd_log[4, :])
    # plt.plot(ref_forces[4, :], linestyle='--')
    plt.ylabel(r"$(tau_{RF})_y$", fontsize=10)

    plt.subplot(6, 2, 6)
    plt.plot(p.tau_ffwd_log[5, :])
    # plt.plot(ref_forces[5, :], linestyle='--')
    plt.ylabel(r"$(tau_{RF})_z$", fontsize=10)

    plt.subplot(6, 2, 7)
    plt.plot(p.tau_ffwd_log[6, :])
    # plt.plot(ref_forces[6, :], linestyle='--')
    plt.ylabel(r"$(tau_{LH})_x$", fontsize=10)

    plt.subplot(6, 2, 9)
    plt.plot(p.tau_ffwd_log[7, :])
    # plt.plot(ref_forces[7, :], linestyle='--')
    plt.ylabel(r"$(tau_{LH})_y$", fontsize=10)

    plt.subplot(6, 2, 11)
    plt.plot(p.tau_ffwd_log[8, :])
    # plt.plot(ref_forces[8, :], linestyle='--')
    plt.ylabel(r"$(tau_{LH})_z$", fontsize=10)

    plt.subplot(6, 2, 8)
    plt.plot(p.tau_ffwd_log[9, :])
    # plt.plot(ref_forces[9, :], linestyle='--')
    plt.ylabel(r"$(tau_{RH})_x$", fontsize=10)

    plt.subplot(6, 2, 10)
    plt.plot(p.tau_ffwd_log[10, :])
    # plt.plot(ref_forces[10, :], linestyle='--')
    plt.ylabel(r"$(tau_{RH})_y$", fontsize=10)

    plt.subplot(6, 2, 12)
    plt.plot(p.tau_ffwd_log[11, :])
    # plt.plot(ref_forces[11, :], linestyle='--')
    plt.ylabel("$(tau_{RH})_z$", fontsize=10)
    plt.show()


def plotJoints(q, q_des):
    plt.figure()
    plt.title("Joints")
    plt.subplot(2, 1, 1)
    plt.plot(q[1, :],  label="Actual", color='g')
    plt.plot(q_des[1, :], label="Desired", linestyle='--')
    plt.ylabel(r"$q_{LF-HFE}$", fontsize=10)
    plt.grid()
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(q[2, :],  label="Actual", color='g')
    plt.plot(q_des[2, :], label="Desired", linestyle='--')
    plt.grid()
    plt.ylabel(r"$q_{LF-KFE}$", fontsize=10)


def plotCones(p, des_forces, opticlass):
    # this is a cross check for the friction cone constraints
    mu_x = np.zeros(opticlass.N)
    mu_y = np.zeros(opticlass.N)
    for i in range(opticlass.N):
        mu_x[i] = des_forces[0, i] / des_forces[2, i]
        mu_y[i] = des_forces[1, i] / des_forces[2, i]
    plt.figure(1)
    plt.plot(mu_x, label="actual mu")
    plt.plot(np.ones(opticlass.N) * opticlass.mu[p.u.leg_map("LF")], label="desired mu", linestyle='--')
    plt.legend()
    plt.ylabel("MU X LF", fontsize=10)

    plt.figure(2)
    plt.plot(mu_y, label="actual mu")
    plt.plot(np.ones(opticlass.N) * opticlass.mu[p.u.leg_map("LF")], label="desired mu", linestyle='--')
    plt.legend()
    plt.ylabel("MU Y LF", fontsize=10)

def plotOptimizationOutput(data):
    model_type = data["model_type"]
    Ts = data["Ts"]
    simTime = data["simTime"]
    des_controls = data["des_controls"]
    ref_controls = data["ref_controls"]
    des_states = data["des_states"]
    ref_states = data["ref_states"]
    des_foot_pos = data["des_foot_pos"]
    ref_foot_pos = data["feet_positionW"]
    des_foot_vel = data["des_foot_vel"]
    des_controls_dot = data["des_controls_dot"]
    legend = data["legend"]
    active_plots = data["active_plots"]
    des_joint_pos = data["des_joint_pos"]
    ref_joint_pos = data["ref_joint_pos"]
    des_joint_vel = data["des_joint_vel"]
    ref_joint_vel = data["ref_joint_vel"]

    plt.rcParams['axes.grid'] = True
    plt.close('all')

    # %% Input plots
    plt.rcParams['axes.grid'] = True
    plt.close('all')

    legend_desired = legend[0]
    legend_ref = legend[1]
    lw_des = 2
    lw_act = 2

    # Forces
    if active_plots.grforces:
        fig = plt.figure()
        fig.suptitle("Ground reaction forces", fontsize=20)
        labels = ["LF x", "LF y", "LF z", "RF x", "RF y", "RF z", "LH x", "LH y", "LH z", "RH x", "RH y", "RH z"]
        idx_vector = [1, 3, 5, 2, 4, 6, 7, 9, 11, 8, 10, 12]

        for jidx in range(12):
            plt.subplot(6, 2, idx_vector[jidx])
            plt.ylabel(labels[jidx])
            plt.plot(simTime, des_controls[jidx, :], linestyle='-', color='red', label=legend_desired)
            plt.plot(simTime, ref_controls[jidx, :], linestyle='--', color='green', label=legend_ref)
            plt.grid(True)

            if (jidx ==1 or jidx ==4):
                # plt.legend()
                plt.legend(bbox_to_anchor=(0., 2.4, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

    # %% COM position and speed
    if active_plots.com:
        xTime = np.append(simTime[:], simTime[-1] + Ts)
        fig = plt.figure()
        fig.suptitle("Com", fontsize=20)
        labels = ["com X", "com Y", "com Z"]
        idx_subplots = [1, 3, 5]
        idx_states = [0, 1, 2]

        for idx in range(3):
            plt.subplot(3, 2, idx_subplots[idx])
            plt.ylabel(labels[idx])
            plt.plot(xTime, des_states[idx_states[idx], :], linestyle='-', lw=lw_des, color='red',
                     label=legend_desired)
            plt.plot(xTime, ref_states[idx_states[idx], :], linestyle='--', lw=lw_act, color='green',
                     label=legend_ref)
            plt.grid(True)

            if (idx == 0):
                # plt.legend()
                plt.legend(bbox_to_anchor=(0., 1.06, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

        labels = ["com vel X", "com vel Y", "com vel Z"]
        idx_subplots = [2, 4, 6]
        idx_states = [3, 4, 5]

        for idx in range(3):
            plt.subplot(3, 2, idx_subplots[idx])
            plt.ylabel(labels[idx])
            plt.plot(xTime, des_states[idx_states[idx], :], linestyle='-', lw=lw_des, color='red',
                     label=legend_desired)
            plt.plot(xTime, ref_states[idx_states[idx], :], linestyle='--', lw=lw_act, color='green',
                     label=legend_ref)
            plt.grid(True)

            if (idx == 0):
                plt.legend(bbox_to_anchor=(0., 1.06, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

    # %% COM angular position and speed
    if active_plots.orientation:
        fig = plt.figure()
        fig.suptitle("Trunk Orientation", fontsize=20)
        labels = ["roll", "pitch", "yaw"]
        idx_subplots = [1, 3, 5]
        idx_states = [6, 7, 8]

        for idx in range(3):
            plt.subplot(3, 2, idx_subplots[idx])
            plt.ylabel(labels[idx])
            plt.plot(xTime, des_states[idx_states[idx], :], linestyle='-', lw=lw_des, color='red',
                     label=legend_desired)
            plt.plot(xTime, ref_states[idx_states[idx], :], linestyle='--', lw=lw_act, color='green',
                     label=legend_ref)
            plt.grid(True)

            if (idx == 0):
                # plt.legend()
                plt.legend(bbox_to_anchor=(0., 1.06, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

        labels = ["omega X", "omega Y", "omega Z"]
        idx_subplots = [2, 4, 6]
        idx_states = [9, 10, 11]

        for idx in range(3):
            plt.subplot(3, 2, idx_subplots[idx])
            plt.ylabel(labels[idx])
            plt.plot(xTime, des_states[idx_states[idx], :], linestyle='-', lw=lw_des, color='red',
                     label=legend_desired)
            plt.plot(xTime, ref_states[idx_states[idx], :], linestyle='--', lw=lw_act, color='green',
                     label=legend_ref)
            plt.grid(True)

            if (idx == 0):
                plt.legend(bbox_to_anchor=(0., 1.06, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

    # Feet position
    if active_plots.feet:
        fig = plt.figure()
        fig.suptitle("Feet Positions", fontsize=20)
        labels = ["LF x", "LF y", "LF z", "RF x", "RF y", "RF z", "LH x", "LH y", "LH z", "RH x",
                  "RH y", "RH z", "Replanning", "Replanning"]
        idx_vector = [1, 3, 5, 2, 4, 6, 7, 9, 11, 8, 10, 12]

        for jidx in range(12):
            plt.subplot(7, 2, idx_vector[jidx])
            plt.ylabel(labels[jidx])
            plt.plot(des_foot_pos[jidx, :].T, linestyle='-', lw=lw_des, color='red', label=legend_desired)
            plt.plot(ref_foot_pos[jidx, :].T, linestyle='--', lw=lw_act, color='green',
                     label=legend_ref)
            plt.grid(True)

            if (jidx == 1 or jidx == 4):
                # plt.legend()
                plt.legend(bbox_to_anchor=(0., 2.4, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

        fig = plt.figure()
        fig.suptitle("Feet Velocities", fontsize=20)
        labels = ["LF x", "LF y", "LF z", "RF x", "RF y", "RF z", "LH x", "LH y", "LH z", "RH x",
                  "RH y", "RH z"]

        for jidx in range(12):
            plt.subplot(7, 2, idx_vector[jidx])
            plt.ylabel(labels[jidx])
            plt.plot(des_foot_vel[jidx, :].T, linestyle='-', lw=lw_des, color='red',
                     label=legend_desired)
            # plt.plot(ref_foot_vel[jidx, :].T, linestyle='--', lw=lw_act, color='green',
            #          label=legend_ref)
            plt.grid(True)

            if (jidx == 1 or jidx == 4):
                plt.legend(bbox_to_anchor=(0., 2.4, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

    # %% Joint angle and angular velocity
    if active_plots.joint:
        fig = plt.figure()
        fig.suptitle("Joint angles", fontsize=20)
        labels = ["LF:HAA", "LF:HFE", "LF:KFE",
                  "RF:HAA", "RF:HFE", "RF:KFE",
                  "LH:HAA", "LH:HFE", "LH:KFE",
                  "RH:HAA", "RH:HFE", "RH:KFE"]
        idx_vector = [1, 3, 5, 2, 4, 6, 7, 9, 11, 8, 10, 12]

        for jidx in range(12):
            plt.subplot(7, 2, idx_vector[jidx])
            plt.ylabel(labels[jidx])
            plt.plot(des_joint_pos[jidx, :].T, linestyle='-', lw=lw_des, color='red',
                     label=legend_desired)
            plt.plot(ref_joint_pos[jidx, :].T, linestyle='--', lw=lw_act, color='green',
                     label=legend_ref)
            plt.grid(True)

            if (jidx == 1 or jidx == 4):
                # plt.legend()
                plt.legend(bbox_to_anchor=(0., 2.4, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

        fig = plt.figure()
        fig.suptitle("Joint AngVel", fontsize=20)
        labels = ["LF:HAA", "LF:HFE", "LF:KFE",
                  "RF:HAA", "RF:HFE", "RF:KFE",
                  "LH:HAA", "LH:HFE", "LH:KFE",
                  "RH:HAA", "RH:HFE", "RH:KFE"]
        idx_vector = [1, 3, 5, 2, 4, 6, 7, 9, 11, 8, 10, 12]

        for jidx in range(12):
            plt.subplot(7, 2, idx_vector[jidx])
            plt.ylabel(labels[jidx])
            plt.plot(des_joint_vel[jidx, :].T, linestyle='-', lw=lw_des, color='red',
                     label=legend_desired)
            plt.plot(ref_joint_vel[jidx, :].T, linestyle='--', lw=lw_act, color='green',
                     label=legend_ref)
            plt.grid(True)

            if (jidx == 1 or jidx == 4):
                # plt.legend()
                plt.legend(bbox_to_anchor=(0., 2.4, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)

    # Input regulation [delta_u] plots
    if model_type == 2 and active_plots.delta_u:

        fig = plt.figure()
        fig.suptitle("Inputs regulation du/dt", fontsize=20)
        labels = ["LF x", "LF y", "LF z", "RF x", "RF y", "RF z", "LH x", "LH y", "LH z", "RH x",
                  "RH y", "RH z"]
        idx_vector = [1, 3, 5, 2, 4, 6, 7, 9, 11, 8, 10, 12]

        for jidx in range(12):
            plt.subplot(6, 2, idx_vector[jidx])
            plt.ylabel(labels[jidx])
            plt.plot(simTime, des_controls_dot[jidx, :], linestyle='-', color='red', label=legend_desired)
            if (jidx == 1 or jidx == 4):
                # plt.legend()
                plt.legend(bbox_to_anchor=(0., 2.4, 1., .102), loc=5,
                           ncol=3, mode="expand", borderaxespad=0.)
    plt.ioff()
    plt.show()

def plotReference(refClass, active_plots):

    plt.close('all')

    number_of_samples = refClass.prediction_horizon

    time = np.zeros(len(refClass.response.time_parametrization))
    accumulated_time = 0
    for i in range(len(refClass.response.time_parametrization)):
        time[i] = accumulated_time
        accumulated_time += refClass.response.time_parametrization[i]

    if active_plots.swing:
        plt.figure()
        plt.rcParams['axes.grid'] = True
        plt.plot(time, np.transpose(refClass.swing[0,:]) * 0.25, linestyle='--', marker='o',label="LF")
        plt.plot(time, np.transpose(refClass.swing[1,:]) * 0.5, linestyle='--', marker='o',label="RF")
        plt.plot(time, np.transpose(refClass.swing[2,:]) * 0.75,linestyle='--', marker='o', label="LH")
        plt.plot(time, np.transpose(refClass.swing[3,:]) * 1, linestyle='--', marker='o',label="RH")
        plt.legend()
        plt.title('Swing vector: reference')
        plt.show()
    #
    if active_plots.grforces:
        plt.figure()
        plt.rcParams['axes.grid'] = True
        plt.plot(time, np.transpose(refClass.grForcesLFWz_gt), marker='o', label="LF")
        plt.plot(time, np.transpose(refClass.grForcesRFWz_gt), marker='o', label="RF")
        plt.plot(time, np.transpose(refClass.grForcesLHWz_gt), marker='o', label="LH")
        plt.plot(time, np.transpose(refClass.grForcesRHWz_gt), marker='o', label="RH")
        plt.legend()
        plt.title('Ground reaction force: reference')
        plt.show()

    if active_plots.feet:
        plt.figure()
        plt.rcParams['axes.grid'] = True
        plt.plot(time, np.transpose(refClass.footPosLFWx), label="LF")
        plt.plot(time, np.transpose(refClass.footPosRFWx), label="RF")
        plt.plot(time, np.transpose(refClass.footPosLHWx), label="LH")
        plt.plot(time,  np.transpose(refClass.footPosRHWx), label="RH")
        plt.legend()
        plt.title('Foot location: reference x')
        plt.show()

        plt.figure()
        #plt.subplot(1, 2, 1)
        plt.rcParams['axes.grid'] = True
        plt.plot(time, np.transpose(refClass.footPosLFWy), label="LF")
        plt.plot(time, np.transpose(refClass.footPosRFWy), label="RF")
        plt.plot(time, np.transpose(refClass.footPosLHWy), label="LH")
        plt.plot(time, np.transpose(refClass.footPosRHWy), label="RH")
        plt.legend()
        plt.title('Foot location: reference y')
        plt.show()

        plt.figure()
        # plt.subplot(1, 2, 1)
        plt.rcParams['axes.grid'] = True
        plt.plot(time, np.transpose(refClass.footPosLFWz), label="LF")
        plt.plot(time, np.transpose(refClass.footPosRFWz), label="RF")
        plt.plot(time, np.transpose(refClass.footPosLHWz), label="LH")
        plt.plot(time, np.transpose(refClass.footPosRHWz), label="RH")
        plt.legend()
        plt.title('Foot location: reference z')
        plt.show()

    if active_plots.com:
        plt.figure()
        plt.rcParams['axes.grid'] = True
        plt.plot(time, np.transpose(refClass.actual_CoMXW),label="X", marker='.', linewidth=1.)
        plt.plot(time, np.transpose(refClass.actual_CoMYW),label="Y",marker='.',  linewidth=2.)
        plt.plot(time, np.transpose(refClass.actual_CoMZW),label="Z",marker='.',  linewidth=3.)
        plt.title('COM position: reference')
        plt.legend()
        plt.show()

        plt.figure()
        plt.rcParams['axes.grid'] = True
        plt.plot(time, np.transpose(refClass.com_VxW),label="X", marker='.',  linewidth=1.)
        plt.plot(time, np.transpose(refClass.com_VyW),label="Y", marker='.', linewidth=2.)
        plt.plot(time, np.transpose(refClass.com_VzW),label="Z", marker='.', linewidth=3.)
        plt.title('COM velocity: reference')
        plt.legend()
        plt.show()

    if active_plots.orientation:
        plt.figure()
        plt.rcParams['axes.grid'] = True
        plt.plot(time, np.transpose(refClass.rollW),label="X", marker='.', linewidth=1.)
        plt.plot(time,  np.transpose(refClass.pitchW),label="Y",marker='.',  linewidth=2.)
        plt.plot(time, np.transpose(refClass.yawW),label="Z",  marker='.',linewidth=3.)
        plt.title('Orientation position: reference')
        plt.legend()
        plt.show()

        plt.figure()
        plt.rcParams['axes.grid'] = True
        plt.plot(time, np.transpose(refClass.omegaXW),label="X",  linewidth=1.)
        plt.plot(time, np.transpose(refClass.omegaYW),label="Y",  linewidth=2.)
        plt.plot(time, np.transpose(refClass.omegaZW),label="Z",  linewidth=3.)
        plt.title('Orientation velocity: reference')
        plt.legend()
        plt.show()

def plot_cone_incontact_frame(mu_mpc, mu_terrain, forces):
    plt.figure()
    plt.plot(forces[2, :], label="$u_{LF_z}$")
    plt.plot(mu_mpc * forces[2, :], label="$\mu * u_{LF_z}$")
    plt.plot(forces[0, :], label="$u_{LF_x}$")
    plt.plot(forces[1, :], label="$u_{LF_y}$")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.scatter(abs(forces[1, :]), forces[2, :])
    fz_max = np.max(forces[2, :])
    plt.plot(np.linspace(0, mu_mpc*fz_max, 100), np.linspace(0, fz_max, 100))
    plt.plot(np.linspace(0, mu_terrain * fz_max, 100), np.linspace(0, fz_max, 100))
    plt.grid(True)
    plt.show(block=True)
