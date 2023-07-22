import numpy as np
import pinocchio as pin
import time
from base_controllers.utils.utils import Utils

class LegOdometry:
    def __init__(self, robot, real_robot = False):
        self.robot = robot
        self.real_robot = real_robot

        self.u = Utils()

        self.w_feet_pos_init = np.empty([3, len(self.robot.getEndEffectorsFrameId)]) * np.nan

        self.w_p_b_update = np.zeros(3)
        self.w_v_b_update = np.zeros(3)

        self.w_p_b = np.zeros(3)
        self.w_v_b = np.zeros(3)

        self._b_conf_neutral = pin.neutral(self.robot.model)
        self._b_vel_neutral  = np.zeros(self.robot.model.nv)

        self._reset_has_been_called_once = False



    def compute_feet_position(self, q):
        pin.forwardKinematics(self.robot.model, self.robot.data, q)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        for i, index in enumerate(self.robot.getEndEffectorsFrameId):
            self.w_feet_pos_init[:, i] = self.robot.data.oMf[index].translation.copy()
            self.w_feet_pos_init[2, i] += 0.02                                          # this is foot radius

    def reset(self, q):
        self.compute_feet_position(q)
        self._reset_has_been_called_once = True

    # deprecated
    def estimate_base_wrt_world(self, contacts_state, quat, qj, ang_vel, vj):
        '''
        get base position (and velocity) given legs joints configuration (and velocity)
        assuming feet haven't changed position
        args: contacts_state, qj, and vj uses ros convention [LF, LH, RF, RH]
        '''
        assert self._reset_has_been_called_once, "Please call reset funcion after initializing LegOdometry"
        w_R_b = pin.Quaternion(quat).toRotationMatrix().T
        w_omega_b = ang_vel
        self._b_conf_neutral[7:] = self.u.mapFromRos(qj)
        self._b_vel_neutral[6:]  = self.u.mapFromRos(vj)


        self.robot.forwardKinematics(self._b_conf_neutral, self._b_vel_neutral)
        pin.updateFramePlacements(self.robot.model, self.robot.data)

        self.w_p_b_update[:] = 0.
        self.w_v_b_update[:] = 0.

        nc = 0
        for k, value in enumerate(contacts_state):
            if value:
                nc+=1
                footid = self.robot.getEndEffectorsFrameId[k]
                b_p_f = self.robot.data.oMf[footid].translation.copy()
                w_p_b = self.w_feet_pos_init[:, k] - w_R_b @ b_p_f

                b_Jl_f = pin.computeFrameJacobian(self.robot.model,
                                                  self.robot.data,
                                                  self._b_conf_neutral,
                                                  footid,
                                                  pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[0:3,:]

                b_v_f = b_Jl_f @ self._b_vel_neutral
                w_v_b = -pin.skew(w_omega_b) @ w_R_b @ b_p_f - w_R_b @ b_v_f

                self.w_p_b_update = (k * self.w_p_b_update + w_p_b)/(k+1)
                self.w_v_b_update = (k * self.w_v_b_update + w_v_b)/(k+1)

        # if nc != 0:
        #     self.w_p_b_update/=nc
        #     self.w_v_b_update/=nc

        return self.w_p_b_update, self.w_v_b_update


    def base_in_world(self, contact_state,  B_contacts, b_R_w, wJ, ang_vel, qd, update_legOdom=True):
        '''
        same idea of the above, but faster. to be used togeter with Controller in quadruped_controller.py
        '''

        if update_legOdom:
            if  self._reset_has_been_called_once:
                nc = 0
                if any(contact_state):
                    self.w_p_b_update[:] = 0.
                    self.w_v_b_update[:] = 0.
                    if self.real_robot == False:
                        for k, value in enumerate(contact_state):
                            if value:
                                nc+=1
                                w_p_b_foot = self.w_feet_pos_init[:, k] - b_R_w@ B_contacts[k]
                                w_v_b_foot = -pin.skew(ang_vel) @  b_R_w @ B_contacts[k] - wJ[k] @ self.u.getLegJointState(k, qd)

                                self.w_p_b_update += w_p_b_foot
                                self.w_v_b_update += w_v_b_foot
                    else:
                        for k, value in enumerate(contact_state):
                            nc = 4
                            w_p_b_foot = self.w_feet_pos_init[:, k] - b_R_w @ B_contacts[k]
                            w_v_b_foot = -pin.skew(ang_vel) @ b_R_w @ B_contacts[k] - wJ[k] @ self.u.getLegJointState(k,
                                                                                                                      qd)

                            self.w_p_b_update += w_p_b_foot
                            self.w_v_b_update += w_v_b_foot

                    self.w_p_b = self.w_p_b_update/nc
                    self.w_v_b = self.w_v_b_update/nc


        return self.w_p_b, self.w_v_b



if __name__ == '__main__':
    from base_controllers.utils.common_functions import getRobotModel

    robot = getRobotModel('solo')
    q0 = np.array([ 0., 0., 0.223,
                    0., 0., 0., 1.,
                    0.,  np.pi / 4, -np.pi / 2,   # lf
                    0., -np.pi / 4,  np.pi / 2,   # lh
                   -0.,  np.pi / 4, -np.pi / 2,   # rf
                   -0., -np.pi / 4,  np.pi / 2])  # rh

    lo = LegOdometry(robot)
    lo.reset(q0)
    quat = q0[3:7]
    qj = q0[7:]
    ang_vel = np.zeros(3)
    vj = np.zeros(robot.na)
    contacts_state = [True, True, True, True] # [LF, LH, RF, RH]

    lin_pos, lin_vel= lo.estimate_base_wrt_world(contacts_state, quat, qj, ang_vel, vj)
    print('base position:', lin_pos)
    print('base velocity:', lin_vel)
