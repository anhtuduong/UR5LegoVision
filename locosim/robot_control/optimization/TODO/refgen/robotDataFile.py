# -*- coding: utf-8 -*-
"""
This is to create data file for usage in python
Created on Tue Apr  11 11:29:11 2019
@author: Niraj Rathod
"""
import numpy as np

class extractROSdata:
    def __init__(self, dataRos):

        # self.simStart = simStart

        # COM position (World frame)
        self.actual_CoMXW = dataRos['actual_CoMXW'][:].T
        self.actual_CoMYW = dataRos['actual_CoMYW'][:].T
        self.actual_CoMZW = dataRos['actual_CoMZW'][:].T

        # COM velocity (World frame)
        self.com_VxW = dataRos['com_VxW'][:]
        self.com_VyW = dataRos['com_VyW'][:]
        self.com_VzW = dataRos['com_VzW'][:]

        # Base orientation (Euler angles)
        self.rollW = dataRos['rollW'][:].T
        self.pitchW = dataRos['pitchW'][:].T
        self.yawW = dataRos['yawW'][:].T

        # Angular velocity of a robot (World frame)
        self.omegaXW = dataRos['omegaXW'][:]
        self.omegaYW = dataRos['omegaYW'][:]
        self.omegaZW = dataRos['omegaZW'][:]

        # Foot locations (World frame)
        self.footPosLFWx = dataRos['footPosLFWx'][:].T
        self.footPosLFWy = dataRos['footPosLFWy'][:].T
        self.footPosLFWz = dataRos['footPosLFWz'][:].T
        self.footPosLHWx = dataRos['footPosLHWx'][:].T
        self.footPosLHWy = dataRos['footPosLHWy'][:].T
        self.footPosLHWz = dataRos['footPosLHWz'][:].T
        self.footPosRFWx = dataRos['footPosRFWx'][:].T
        self.footPosRFWy = dataRos['footPosRFWy'][:].T
        self.footPosRFWz = dataRos['footPosRFWz'][:].T
        self.footPosRHWx = dataRos['footPosRHWx'][:].T
        self.footPosRHWy = dataRos['footPosRHWy'][:].T
        self.footPosRHWz = dataRos['footPosRHWz'][:].T

        # Ground reaction forces (World frame)
        self.grForcesLFWx_gt= dataRos['grForcesLFWx_gt'][:].T
        self.grForcesLFWy_gt= dataRos['grForcesLFWy_gt'][:].T
        self.grForcesLFWz_gt= dataRos['grForcesLFWz_gt'][:].T
        self.grForcesLHWx_gt= dataRos['grForcesLHWx_gt'][:].T
        self.grForcesLHWy_gt= dataRos['grForcesLHWy_gt'][:].T
        self.grForcesLHWz_gt= dataRos['grForcesLHWz_gt'][:].T
        self.grForcesRFWx_gt= dataRos['grForcesRFWx_gt'][:].T
        self.grForcesRFWy_gt= dataRos['grForcesRFWy_gt'][:].T
        self.grForcesRFWz_gt= dataRos['grForcesRFWz_gt'][:].T
        self.grForcesRHWx_gt= dataRos['grForcesRHWx_gt'][:].T
        self.grForcesRHWy_gt= dataRos['grForcesRHWy_gt'][:].T
        self.grForcesRHWz_gt= dataRos['grForcesRHWz_gt'][:].T

        # self.range = dataRos['range'][:]
        self.simTime = dataRos['simTime'][:].T
        # self.state_machine = dataRos['state_machine'][:]

        # Swing vectors (1 -> swing)
        self.swingLF = dataRos['swingLF'][:]
        self.swingLH = dataRos['swingLH'][:]
        self.swingRF = dataRos['swingRF'][:]
        self.swingRH = dataRos['swingRH'][:]

        # Stance vectors (1 -> stance)
        if 'stanceLF' in dataRos.keys():
            self.stanceLF = dataRos['stanceLF'][:]
        if 'stanceRF' in dataRos.keys():
            self.stanceRF = dataRos['stanceRF'][:]
        if 'stanceLH' in dataRos.keys():
            self.stanceLH = dataRos['stanceLH'][:]
        if 'stanceRH' in dataRos.keys():
            self.stanceRH = dataRos['stanceRH'][:]

        # Desired COM position (world frame) & orientation (Euler angles)
        if 'des_CoMXW' in dataRos.keys():
            self.des_CoMXW = dataRos['des_CoMXW'][:]

        if 'des_CoMYW' in dataRos.keys():
            self.des_CoMYW = dataRos['des_CoMYW'][:]

        if 'des_CoMZW' in dataRos.keys():
            self.des_CoMZW = dataRos['des_CoMZW'][:]

        if 'des_rollW' in dataRos.keys():
            self.des_rollW = dataRos['des_rollW'][:]

        if 'des_pitchW' in dataRos.keys():
            self.des_pitchW = dataRos['des_pitchW'][:]

        if 'des_yawW' in dataRos.keys():
            self.des_yawW = dataRos['des_yawW'][:]


        # Desired COM velocity & angular velocity (world frame)
        if 'des_com_VxW' in dataRos.keys():
            self.des_com_VxW = dataRos['des_com_VxW'][:]

        if 'des_com_VyW' in dataRos.keys():
            self.des_com_VyW = dataRos['des_com_VyW'][:]

        if 'des_com_VzW' in dataRos.keys():
            self.des_com_VzW = dataRos['des_com_VzW'][:]

        if 'des_omegaXW' in dataRos.keys():
            self.des_omegaXW = dataRos['des_omegaXW'][:]

        if 'des_omegaYW' in dataRos.keys():
            self.des_omegaYW = dataRos['des_omegaYW'][:]

        if 'des_omegaZW' in dataRos.keys():
            self.des_omegaZW = dataRos['des_omegaZW'][:]

        # Desired COM acceleration and angular acceleration (world frame)
        if 'des_com_XddW' in dataRos.keys():
            self.des_com_XddW = dataRos['des_com_XddW'][:]

        if 'des_com_YddW' in dataRos.keys():
            self.des_com_YddW = dataRos['des_com_YddW'][:]

        if 'des_com_ZddW' in dataRos.keys():
            self.des_com_ZddW = dataRos['des_com_ZddW'][:]

        if 'des_omegaXWd' in dataRos.keys():
            self.des_omegaXWd = dataRos['des_omegaXWd'][:]

        if 'des_omegaYWd' in dataRos.keys():
            self.des_omegaYWd = dataRos['des_omegaYWd'][:]

        if 'des_omegaZWd' in dataRos.keys():
            self.des_omegaZWd = dataRos['des_omegaZWd'][:]


        # def getInitState(self):
    #     x_ref_traj=np.array([self.actual_CoMXW[0,self.simStart],
    #                          self.actual_CoMYW[0,self.simStart],
    #                          self.actual_CoMZW[0,self.simStart],
    #                          self.com_VxW[0,self.simStart],
    #                          self.com_VyW[0,self.simStart],
    #                          self.com_VzW[0,self.simStart],
    #                          self.rollW[0,self.simStart],
    #                          self.pitchW[0,self.simStart],
    #                          self.yawW[0,self.simStart],
    #                          self.omegaXW[0,self.simStart],
    #                          self.omegaYW[0,self.simStart],
    #                          self.omegaZW[0,self.simStart],
    #                          0,0, -9.81])
    #     return x_ref_traj

class defStruct:
    def __init__(self, rosData, simStart,N):
        # reshape to make it compatible to the class script
        rosData.actual_CoMXW = rosData.actual_CoMXW.flatten()
        rosData.actual_CoMYW = rosData.actual_CoMYW.flatten()
        rosData.actual_CoMZW = rosData.actual_CoMZW.flatten()

        rosData.omegaXW = rosData.omegaXW.flatten()
        rosData.omegaYW = rosData.omegaYW.flatten()
        rosData.omegaZW = rosData.omegaZW.flatten()

        rosData.com_VxW = rosData.com_VxW.flatten()
        rosData.com_VyW = rosData.com_VyW.flatten()
        rosData.com_VzW = rosData.com_VzW.flatten()

        rosData.rollW = rosData.rollW.flatten()
        rosData.pitchW = rosData.pitchW.flatten()
        rosData.yawW = rosData.yawW.flatten()

        rosData.grForcesLFWx_gt = rosData.grForcesLFWx_gt.flatten()
        rosData.grForcesLFWy_gt = rosData.grForcesLFWy_gt.flatten()
        rosData.grForcesLFWz_gt = rosData.grForcesLFWz_gt.flatten()
        rosData.grForcesRFWx_gt = rosData.grForcesRFWx_gt.flatten()
        rosData.grForcesRFWy_gt = rosData.grForcesRFWy_gt.flatten()
        rosData.grForcesRFWz_gt = rosData.grForcesRFWz_gt.flatten()
        rosData.grForcesLHWx_gt = rosData.grForcesLHWx_gt.flatten()
        rosData.grForcesLHWy_gt = rosData.grForcesLHWy_gt.flatten()
        rosData.grForcesLHWz_gt = rosData.grForcesLHWz_gt.flatten()
        rosData.grForcesRHWx_gt = rosData.grForcesRHWx_gt.flatten()
        rosData.grForcesRHWy_gt = rosData.grForcesRHWy_gt.flatten()
        rosData.grForcesRHWz_gt = rosData.grForcesRHWz_gt.flatten()

        rosData.footPosLFWx = rosData.footPosLFWx.flatten()
        rosData.footPosLFWy = rosData.footPosLFWy.flatten()
        rosData.footPosLFWz = rosData.footPosLFWz.flatten()
        rosData.footPosRFWx = rosData.footPosRFWx.flatten()
        rosData.footPosRFWy = rosData.footPosRFWy.flatten()
        rosData.footPosRFWz = rosData.footPosRFWz.flatten()
        rosData.footPosLHWx = rosData.footPosLHWx.flatten()
        rosData.footPosLHWy = rosData.footPosLHWy.flatten()
        rosData.footPosLHWz = rosData.footPosLHWz.flatten()
        rosData.footPosRHWx = rosData.footPosRHWx.flatten()
        rosData.footPosRHWy = rosData.footPosRHWy.flatten()
        rosData.footPosRHWz = rosData.footPosRHWz.flatten()

        self.actual_CoMXW = np.zeros(N)
        self.actual_CoMYW = np.zeros(N)
        self.actual_CoMZW = np.zeros(N)

        self.com_VxW = np.zeros(N)
        self.com_VyW = np.zeros(N)
        self.com_VzW = np.zeros(N)

        self.footPosLFWx = np.zeros(N)
        self.footPosLFWy = np.zeros(N)
        self.footPosLFWz = np.zeros(N)
        self.footPosLHWx = np.zeros(N)
        self.footPosLHWy = np.zeros(N)
        self.footPosLHWz = np.zeros(N)
        self.footPosRFWx = np.zeros(N)
        self.footPosRFWy = np.zeros(N)
        self.footPosRFWz = np.zeros(N)
        self.footPosRHWx = np.zeros(N)
        self.footPosRHWy = np.zeros(N)
        self.footPosRHWz = np.zeros(N)

        self.grForcesLFWx_gt = np.zeros(N)
        self.grForcesLFWy_gt = np.zeros(N)
        self.grForcesLFWz_gt = np.zeros(N)
        self.grForcesLHWx_gt = np.zeros(N)
        self.grForcesLHWy_gt = np.zeros(N)
        self.grForcesLHWz_gt = np.zeros(N)
        self.grForcesRFWx_gt = np.zeros(N)
        self.grForcesRFWy_gt = np.zeros(N)
        self.grForcesRFWz_gt = np.zeros(N)
        self.grForcesRHWx_gt = np.zeros(N)
        self.grForcesRHWy_gt = np.zeros(N)
        self.grForcesRHWz_gt = np.zeros(N)

        self.omegaXW = np.zeros(N)
        self.omegaYW = np.zeros(N)
        self.omegaZW = np.zeros(N)

        self.rollW = np.zeros(N)
        self.pitchW = np.zeros(N)
        self.yawW = np.zeros(N)

        self.simTime = np.zeros(N)
        self.swingLF = np.zeros(N)
        self.swingLH = np.zeros(N)
        self.swingRF = np.zeros(N)
        self.swingRH = np.zeros(N)

        for i in range(simStart, simStart+N):
            self.actual_CoMXW[i-simStart] = rosData.actual_CoMXW[i]
            self.actual_CoMYW[i-simStart] = rosData.actual_CoMYW[i]
            self.actual_CoMZW[i-simStart] = rosData.actual_CoMZW[i]

            self.com_VxW[i-simStart] = rosData.com_VxW[i]
            self.com_VyW[i-simStart] = rosData.com_VyW[i]
            self.com_VzW[i-simStart] = rosData.com_VzW[i]

            self.footPosLFWx[i-simStart] = rosData.footPosLFWx[i]
            self.footPosLFWy[i-simStart] = rosData.footPosLFWy[i]
            self.footPosLFWz[i-simStart] = rosData.footPosLFWz[i]
            self.footPosLHWx[i-simStart] = rosData.footPosLHWx[i]
            self.footPosLHWy[i-simStart] = rosData.footPosLHWy[i]
            self.footPosLHWz[i-simStart] = rosData.footPosLHWz[i]
            self.footPosRFWx[i-simStart] = rosData.footPosRFWx[i]
            self.footPosRFWy[i-simStart] = rosData.footPosRFWy[i]
            self.footPosRFWz[i-simStart] = rosData.footPosRFWz[i]
            self.footPosRHWx[i-simStart] = rosData.footPosRHWx[i]
            self.footPosRHWy[i-simStart] = rosData.footPosRHWy[i]
            self.footPosRHWz[i-simStart] = rosData.footPosRHWz[i]

            self.grForcesLFWx_gt[i-simStart] = rosData.grForcesLFWx_gt[i]
            self.grForcesLFWy_gt[i-simStart] = rosData.grForcesLFWy_gt[i]
            self.grForcesLFWz_gt[i-simStart] = rosData.grForcesLFWz_gt[i]
            self.grForcesLHWx_gt[i-simStart] = rosData.grForcesLHWx_gt[i]
            self.grForcesLHWy_gt[i-simStart] = rosData.grForcesLHWy_gt[i]
            self.grForcesLHWz_gt[i-simStart] = rosData.grForcesLHWz_gt[i]
            self.grForcesRFWx_gt[i-simStart] = rosData.grForcesRFWx_gt[i]
            self.grForcesRFWy_gt[i-simStart] = rosData.grForcesRFWy_gt[i]
            self.grForcesRFWz_gt[i-simStart] = rosData.grForcesRFWz_gt[i]
            self.grForcesRHWx_gt[i-simStart] = rosData.grForcesRHWx_gt[i]
            self.grForcesRHWy_gt[i-simStart] = rosData.grForcesRHWy_gt[i]
            self.grForcesRHWz_gt[i-simStart] = rosData.grForcesRHWz_gt[i]

            self.omegaXW[i-simStart] = rosData.omegaXW[i]
            self.omegaYW[i-simStart] = rosData.omegaYW[i]
            self.omegaZW[i-simStart] = rosData.omegaZW[i]

            self.rollW[i-simStart] = rosData.rollW[i]
            self.pitchW[i-simStart] = rosData.pitchW[i]
            self.yawW[i-simStart] = rosData.yawW[i]

            self.simTime[i-simStart] = rosData.simTime.transpose()[i]
            self.swingLF[i-simStart] = rosData.swingLF.transpose()[i]
            self.swingLH[i-simStart] = rosData.swingLH.transpose()[i]
            self.swingRF[i-simStart] = rosData.swingRF.transpose()[i]
            self.swingRH[i-simStart] = rosData.swingRH.transpose()[i]

class TextfileToStruct():
    def __init__(self, ref_state_opti_c, ref_force_opti_c,param_opti_c):
        # reshape to make it compatible to the class script
        self.actual_CoMXW = ref_state_opti_c[0, :]
        self.actual_CoMYW = ref_state_opti_c[1, :]
        self.actual_CoMZW = ref_state_opti_c[2, :]

        self.com_VxW = ref_state_opti_c[3, :]
        self.com_VyW = ref_state_opti_c[4, :]
        self.com_VzW = ref_state_opti_c[5, :]

        self.rollW = ref_state_opti_c[6, :]
        self.pitchW = ref_state_opti_c[7, :]
        self.yawW = ref_state_opti_c[8, :]

        self.omegaXW = ref_state_opti_c[9, :]
        self.omegaYW = ref_state_opti_c[10, :]
        self.omegaZW = ref_state_opti_c[11, :]

        self.grForcesLFWx_gt = ref_force_opti_c[0, :]
        self.grForcesLFWy_gt = ref_force_opti_c[1, :]
        self.grForcesLFWz_gt = ref_force_opti_c[2, :]
        self.grForcesRFWx_gt = ref_force_opti_c[3, :]
        self.grForcesRFWy_gt = ref_force_opti_c[4, :]
        self.grForcesRFWz_gt = ref_force_opti_c[5, :]
        self.grForcesLHWx_gt = ref_force_opti_c[6, :]
        self.grForcesLHWy_gt = ref_force_opti_c[7, :]
        self.grForcesLHWz_gt = ref_force_opti_c[8, :]
        self.grForcesRHWx_gt = ref_force_opti_c[9, :]
        self.grForcesRHWy_gt = ref_force_opti_c[10, :]
        self.grForcesRHWz_gt = ref_force_opti_c[11, :]

        self.footPosLFWx = param_opti_c[0, :]
        self.footPosLFWy = param_opti_c[1, :]
        self.footPosLFWz = param_opti_c[2, :]
        self.footPosRFWx = param_opti_c[3, :]
        self.footPosRFWy = param_opti_c[4, :]
        self.footPosRFWz = param_opti_c[5, :]
        self.footPosLHWx = param_opti_c[6, :]
        self.footPosLHWy = param_opti_c[7, :]
        self.footPosLHWz = param_opti_c[8, :]
        self.footPosRHWx = param_opti_c[9, :]
        self.footPosRHWy = param_opti_c[10, :]
        self.footPosRHWz = param_opti_c[11, :]