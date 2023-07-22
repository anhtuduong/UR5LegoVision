/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef WOLF_ROBOT_HW_INTERFACE_H
#define WOLF_ROBOT_HW_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <base_hardware_interface/ground_truth_interface.h>
#include <base_hardware_interface/contact_switch_sensor_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <deque>

namespace hardware_interface
{

class WolfRobotHwInterface
{
public:

    const std::string CLASS_NAME = "WolfRobotHwInterface";

    WolfRobotHwInterface();
    virtual ~WolfRobotHwInterface();

    void initializeJointsInterface(const std::vector<std::string>& joint_names);
    void initializeImuInterface(const std::string& imu_link_name);
    void initializeGroundTruthInterface(const std::string& base_link_name);
    void initializeContactSensorsInterface(const std::vector<std::string>& contact_names);

    std::string getRobotName() {return robot_name_;}
    unsigned int getNdof() {return n_dof_;}

    std::vector<std::string> loadJointNamesFromSRDF();
    std::string loadImuLinkNameFromSRDF();
    std::string loadBaseLinkNameFromSRDF();
    std::vector<std::string> loadContactSensorNamesFromSRDF();

protected:

    std::string robot_name_;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::ImuSensorInterface imu_sensor_interface_;
    hardware_interface::GroundTruthInterface ground_truth_interface_;
    hardware_interface::ContactSwitchSensorInterface contact_sensor_interface_;
    hardware_interface::EffortJointInterface joint_effort_interface_;

    unsigned int n_dof_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> contact_sensor_names_;
    std::vector<int> joint_types_;
    std::vector<double> joint_effort_limits_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_effort_command_;

    hardware_interface::GroundTruthHandle::Data gt_data_;
    std::vector<double> base_orientation_;
    std::vector<double>	base_ang_vel_;
    std::vector<double> base_ang_vel_prev_;
    std::vector<double>	base_ang_acc_;
    std::vector<double>	base_lin_acc_;
    std::vector<double>	base_lin_pos_;
    std::vector<double>	base_lin_vel_;
    std::vector<double> base_lin_vel_prev_;

    hardware_interface::ImuSensorHandle::Data imu_data_;
    std::vector<double> imu_orientation_;
    std::vector<double>	imu_ang_vel_;
    std::vector<double>	imu_lin_acc_;


    std::vector<double>	imu_euler_;

    std::vector<std::string> leg_name_;
    std::vector<std::vector<double> > force_;
    std::vector<std::vector<double> > torque_;
    std::vector<std::vector<double> > normal_;
    std::deque<bool> contact_;


private:

    bool parseSRDF(srdf::Model& srdf_model);
};

} //@namespace hardware_interface

#endif
