import matplotlib
# import PyQt5
# matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

# %% Input plots
def plotOcpResults(simTime, x_pred, x_lin_data, u_opt, u_lin_data):
    plt.rcParams['axes.grid'] = True
    plt.close('all')

    plt.figure(1)
    plt.subplot(6, 2, 1)
    plt.plot(simTime[0:-1], u_opt[0,:], label="Optimal")
    plt.plot(simTime[0:-1], u_lin_data[0, :], label="Reference",
             linestyle='--')
    # plt.legend()
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
               ncol=2, mode="expand", borderaxespad=0.)
    plt.ylabel(r"$(u_{LF})_x$", fontsize=10)

    plt.subplot(6, 2, 3)
    plt.plot(simTime[0:-1], u_opt[1,:])
    plt.plot(simTime[0:-1], u_lin_data[1, :], linestyle='--')
    plt.ylabel(r"$(u_{LF})_y$", fontsize=10)

    plt.subplot(6, 2, 5)
    plt.plot(simTime[0:-1], u_opt[2,:])
    plt.plot(simTime[0:-1], u_lin_data[2, :], linestyle='--')
    plt.ylabel(r"$(u_{LF})_z$", fontsize=10)

    plt.subplot(6, 2, 2)
    plt.plot(simTime[0:-1], u_opt[3,:], label="Optimal")
    plt.plot(simTime[0:-1], u_lin_data[3, :], linestyle='--', label="Reference")
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
               ncol=2, mode="expand", borderaxespad=0.)
    plt.ylabel(r"$(u_{RF})_x$", fontsize=10)

    plt.subplot(6, 2, 4)
    plt.plot(simTime[0:-1], u_opt[4,:])
    plt.plot(simTime[0:-1], u_lin_data[4, :], linestyle='--')
    plt.ylabel(r"$(u_{RF})_y$", fontsize=10)

    plt.subplot(6, 2, 6)
    plt.plot(simTime[0:-1], u_opt[5,:])
    plt.plot(simTime[0:-1], u_lin_data[5, :], linestyle='--')
    plt.ylabel(r"$(u_{RF})_z$", fontsize=10)

    plt.subplot(6, 2, 7)
    plt.plot(simTime[0:-1], u_opt[6,:])
    plt.plot(simTime[0:-1], u_lin_data[6, :], linestyle='--')
    plt.ylabel(r"$(u_{LH})_x$", fontsize=10)

    plt.subplot(6, 2, 9)
    plt.plot(simTime[0:-1], u_opt[7,:])
    plt.plot(simTime[0:-1], u_lin_data[7, :], linestyle='--')
    plt.ylabel(r"$(u_{LH})_y$", fontsize=10)

    plt.subplot(6, 2, 11)
    plt.plot(simTime[0:-1], u_opt[8,:])
    plt.plot(simTime[0:-1], u_lin_data[8, :], linestyle='--')
    plt.ylabel(r"$(u_{LH})_z$", fontsize=10)

    plt.subplot(6, 2, 8)
    plt.plot(simTime[0:-1], u_opt[9,:])
    plt.plot(simTime[0:-1], u_lin_data[9, :], linestyle='--')
    plt.ylabel(r"$(u_{RH})_x$", fontsize=10)

    plt.subplot(6, 2, 10)
    plt.plot(simTime[0:-1], u_opt[10,:])
    plt.plot(simTime[0:-1], u_lin_data[10, :], linestyle='--')
    plt.ylabel(r"$(u_{RH})_y$", fontsize=10)

    plt.subplot(6, 2, 12)
    plt.plot(simTime[0:-1], u_opt[11,:])
    plt.plot(simTime[0:-1], u_lin_data[11, :], linestyle='--')
    plt.ylabel("$(u_{RH})_z$", fontsize=10)
    plt.suptitle('Inputs: Ground reaction forces')
    # plt.show()

    # %% COM position and speed reference and predictions by optimizer

    plt.figure(2)
    plt.subplot(3, 2, 1)
    plt.plot(simTime,x_pred[0, :], label="Predicted")
    plt.plot(simTime[0:-1],x_lin_data[0, :], label="Reference",
             linestyle='--')

    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
               ncol=2, mode="expand", borderaxespad=0.)
    plt.ylabel("$COM_x$", fontsize=10)
    # plt.legend()

    plt.subplot(3, 2, 3)
    plt.plot(simTime,x_pred[1, :])
    plt.plot(simTime[0:-1],x_lin_data[1, :], linestyle='--')
    plt.ylabel("$COM_y$", fontsize=10)

    plt.subplot(3, 2, 5)
    plt.plot(simTime,x_pred[2, :])
    plt.plot(simTime[0:-1],x_lin_data[2, :], linestyle='--')
    plt.ylabel("$COM_z$", fontsize=10)

    plt.subplot(3, 2, 2)
    plt.plot(simTime,x_pred[3, :], label="Predicted")
    plt.plot(simTime[0:-1],x_lin_data[3, :], linestyle='--', label="Reference")
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
               ncol=2, mode="expand", borderaxespad=0.)
    plt.ylabel("$COMV_x$", fontsize=10)

    plt.subplot(3, 2, 4)
    plt.plot(simTime,x_pred[4, :])
    plt.plot(simTime[0:-1],x_lin_data[4, :], linestyle='--')
    plt.ylabel("$COMV_y$", fontsize=10)

    plt.subplot(3, 2, 6)
    plt.plot(simTime,x_pred[5, :])
    plt.plot(simTime[0:-1],x_lin_data[5, :], linestyle='--')
    plt.ylabel("$COMV_z$", fontsize=10)
    plt.suptitle('COM linear position and velocity')
    # plt.show()

    # %% COM angular position and speed reference and predictions by optimizer

    plt.figure(3)
    plt.subplot(3, 2, 1)
    plt.plot(simTime,x_pred[6, :], label="Predicted")
    plt.plot(simTime[0:-1],x_lin_data[6, :], label="Reference",
             linestyle='--')
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
               ncol=2, mode="expand", borderaxespad=0.)
    plt.ylabel("$COM\,roll$", fontsize=10)
    # plt.legend()

    plt.subplot(3, 2, 3)
    plt.plot(simTime,x_pred[7, :])
    plt.plot(simTime[0:-1],x_lin_data[7, :], linestyle='--')
    plt.ylabel("$COM\,pitch$", fontsize=10)

    plt.subplot(3, 2, 5)
    plt.plot(simTime,x_pred[8, :])
    plt.plot(simTime[0:-1],x_lin_data[8, :], linestyle='--')
    plt.ylabel("$COM\,yaw$", fontsize=10)

    plt.subplot(3, 2, 2)
    plt.plot(simTime,x_pred[9, :], label="Predicted")
    plt.plot(simTime[0:-1],x_lin_data[9, :], linestyle='--', label="Reference")
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
               ncol=2, mode="expand", borderaxespad=0.)
    plt.ylabel("$\omega_x$", fontsize=10)

    plt.subplot(3, 2, 4)
    plt.plot(simTime,x_pred[10, :])
    plt.plot(simTime[0:-1],x_lin_data[10, :], linestyle='--')
    plt.ylabel("$\omega_y$", fontsize=10)

    plt.subplot(3, 2, 6)
    plt.plot(simTime,x_pred[11, :])
    plt.plot(simTime[0:-1],x_lin_data[11, :], linestyle='--')
    plt.ylabel("$\omega_z$", fontsize=10)
    plt.suptitle('COM angular position and velocity')
    plt.show()
