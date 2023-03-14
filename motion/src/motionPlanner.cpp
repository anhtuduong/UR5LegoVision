/**
 * @file motionPlanner.cpp
 * @author De Martini Davide (davide.demartini@studenti.unitn.it)
 * @brief  This file contains the motion planner node
 * @version 1
 * @date 2023-02-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ros/ros.h"
#include "kinematics.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include "motion/pos.h"
#include <ros_impedance_controller/generic_float.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <complex>

// ----------- DEFINES ----------- //
/// Loop rate of the node
#define LOOP_RATE 1000
/// Number of joints
#define JOINTS 9

// ----------- NAMESPACES ----------- //

using namespace std;
using namespace Eigen;

// ----------- STRUCTS ----------- //

/**
 * @brief This struct contains the position and orientation of the end-effector
 * 
 */
struct Pose
{
    Vector3f position;
    Vector3f orientation;
};

// ----------- GLOBAL VARIABLES ----------- //
/// Position of the end-effector
Pose pose;
/// Class of the block
int class_id;
/// Flag to check if it have to grasp
int grasp = 0;
/// Publisher for the desired joint state
ros::Publisher pub_des_jstate;
/// Publisher for the ack
ros::Publisher ack_pos;
/// Subscriber for the position msg
ros::Subscriber sub_pos;
/// Service call for the gripper
ros::ServiceClient client;
/// Flag to check if it is in simulation
int real_robot = 0;
/// @brief Initial joint configuration
VectorXf TH0(6);
/// @brief Flag to check if it is the first time that the node is called
int first = 1;
double maxT = 6;


// ----------- FUNCTION PROTOTIPES ----------- //

float mapToGripperJoints(float diameter);
Vector3f computeOrientationErrorW(Matrix3f w_R_e, Matrix3f w_R_d);
VectorXf invDiffKinematicControlComplete(VectorXf q, Vector3f xe, Vector3f xd, Vector3f vd, Matrix3f w_R_e, Vector3f phief);
Vector3f pd(double t, Vector3f xef, Vector3f xe0);
void invDiffKinematicControlSimComplete(Vector3f xef, Vector3f phief, float dt);
void posCallback(const motion::pos::ConstPtr &msg);
void sendJointState(VectorXf q);
void move();
void startingPosition();
void ack();
void graspit();

// ----------- MAIN ----------- //

int main(int argc, char **argv)
{
    // initialize ros, node handler and publisher
    ros::init(argc, argv, "custom_joint_publisher");
    ros::NodeHandle node;

    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    ack_pos = node.advertise<std_msgs::Int32>("/motion/ack", 1);

    client = node.serviceClient<ros_impedance_controller::generic_float>("/move_gripper");

    sub_pos = node.subscribe("/motion/pos", 1, posCallback);

    TH0 << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49; // initial joint configuration

    startingPosition();

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}

// ----------- FUNCTION DEFINITIONS ----------- //

/**
 * @brief This function computes the angles of the gripper joint based on the diameter given
 *
 * @param diameter diameter of the gripper
 * @return Vector3f
 */
float mapToGripperJoints(float diameter)
{
    float alpha = (diameter - 22) / (130 - 22) * (-M_PI) + M_PI;
    return alpha;
}

/**
 * @brief This function computes the error between the current and desired end-effector position
 *
 * @param w_R_d rotation matrix of the desired end-effector orientation
 * @param w_R_e rotation matrix of the current end-effector orientation
 * @return Vector3f
 */
Vector3f computeOrientationErrorW(Matrix3f w_R_e, Matrix3f w_R_d)
{
    // compute relative orientation
    Matrix3f e_R_d = w_R_e.transpose() * w_R_d;

    Vector3f errorW;

    // compute the delta angle
    float cos_dtheta = (e_R_d(0, 0) + e_R_d(1, 1) + e_R_d(2, 2) - 1) / 2;
    Vector3f axis;
    MatrixXf aux(3, 2);
    aux << e_R_d(2, 1), -e_R_d(1, 2),
        e_R_d(0, 2), -e_R_d(2, 0),
        e_R_d(1, 0), -e_R_d(0, 1);
    float sin_dtheta = (pow(aux(0, 0), 2) + pow(aux(0, 1), 2) + pow(aux(1, 0), 2) + pow(aux(1, 1), 2) + pow(aux(2, 0), 2) + pow(aux(2, 1), 2)) * 0.5;

    float dtheta = atan2(sin_dtheta, cos_dtheta);
    if (dtheta == 0)
    {
        errorW << 0, 0, 0;
    }
    else
    {

        axis = 1 / (2 * sin_dtheta) * Vector3f(e_R_d(2, 1) - e_R_d(1, 2), e_R_d(0, 2) - e_R_d(2, 0), e_R_d(1, 0) - e_R_d(0, 1));
        errorW = w_R_e * dtheta * axis;
    }

    return errorW;
}

/**
 * @brief      This function is used to calculate the joint velocities using the jacobian matrix
 *
 * @param[in]  q     The current joint config
 * @param[in]  xe    The current end-effector position
 * @param[in]  xd    The desired end-effector position
 * @param[in]  vd    The desired end-effector linear velocity
 * @param[in]  w_R_e The current end-effector orientation
 * @param[in]  phief The desired end-effector orientation
 *
 * @return     The joint velocities
 */
VectorXf invDiffKinematicControlComplete(VectorXf q, Vector3f xe, Vector3f xd, Vector3f vd, Matrix3f w_R_e, Vector3f phief)
{
    float k = pow(10, -5); // damping coefficient
    Matrix3f w_R_d = eul2rotm(phief);
    Vector3f error_o = computeOrientationErrorW(w_R_e, w_R_d);
    MatrixXf J;      // jacobian matrix
    J = jacobian(q); // get the jacobian matrix
    VectorXf qdot;
    VectorXf ve(6);

    Matrix3f kp; // position gain
    kp = Matrix3f::Identity() * 5;

    Matrix3f kphi; // orientation gain
    kphi = Matrix3f::Identity() * 30;

    if (error_o.norm() > 1)
    {
        error_o = 0.1 * error_o.normalized();
    }

    ve << (vd + kp * (xd - xe)), (kphi * error_o);
    qdot = (J + MatrixXf::Identity(6, 6) * k).inverse() * ve;

    for (int i = 0; i < 6; i++)
    {
        if (qdot(i) > M_PI)
        {
            qdot(i) = 1.5;
        }
        else if (qdot(i) < -M_PI)
        {
            qdot(i) = -1.5;
        }
    }

    return qdot;
}

/**
 * @brief      This function is used to calculate trajectory for the end-effector position
 *
 * @param[in]  t     The current time
 * @param[in]  xef    The desired end-effector position
 * @param[in]  xe0    The start end-effector position
 *
 * @return     The end-effector position
 */
Vector3f pd(double t, Vector3f xef, Vector3f xe0)
{
    double t_norm = t / maxT;
    if (t_norm > 1)
    {
        return xef;
    }
    else
    {
        return t_norm * xef + (1 - t_norm) * xe0;
    }
}

/**
 * @brief           This function is used to calculate the joint config matrix using the inverse differential kinematics
 *
 * @param xef       desired end-effector position
 * @param phief     desired end-effector orientation
 * @param dt        time step
 * @return MatrixXf joint config matrix
 */
void invDiffKinematicControlSimComplete(Vector3f xef, Vector3f phief, float dt)
{
    frame now;   // current frame
    frame start; // start frame
    if  (first)
        TH0 << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49; // initial joint configuration
    first = 0;
    start = directKin(TH0);

    VectorXf qk = TH0;           // set the initial joint config
    VectorXf qk1(6);             // joint config

    Matrix3f kp; // position gain
    kp = Matrix3f::Identity() * 5;

    Matrix3f kphi; // orientation gain
    kphi = Matrix3f::Identity() * 30;

    VectorXf dotqk(6); // joint velocities coefficients

    Vector3f vd; // desired linear velocity                                                                                                                  // get the inverse kinematics matrix

    // loop
    for (double i = dt; i <= maxT; i += dt)
    {
        now = directKin(qk); // get the current frame

        vd = (pd(i, xef, start.xyz) - pd(i - dt, xef, start.xyz)) / dt; // desired linear velocity

        // coefficient
        dotqk = invDiffKinematicControlComplete(qk, now.xyz, pd(i, xef, start.xyz), vd, now.rot, phief);

        // euler integration
        qk1 = qk + dotqk * dt;
        qk = qk1;
        sendJointState(qk);
    }
    TH0 = qk;
}

/**
 * @brief CALLBACK function for the position topic
 * 
 * @param msg message received
 */
void posCallback(const motion::pos::ConstPtr &msg)
{
    pose.position(0) = msg->x;
    pose.position(1) = msg->y;
    pose.position(2) = msg->z;

    pose.orientation(0) = msg->roll;
    pose.orientation(1) = msg->pitch;
    pose.orientation(2) = msg->yaw;

    class_id = msg->class_id;

    move();
}

/**
 * @brief      This function is used to send the joint states to the robot
 *
 * @param[in]  q     The joint config
 */
void sendJointState(VectorXf q)
{
    ros::Rate loop_rate(LOOP_RATE);
    std_msgs::Float64MultiArray jointState_msg_robot;
    jointState_msg_robot.data.resize(JOINTS);
    for (int i = 0; i < 6; i++)
    {
        jointState_msg_robot.data[i] = q(i);
    }
    jointState_msg_robot.data[5] = 3.49;
    if (!real_robot)
    {
        if (grasp)
        {            
            jointState_msg_robot.data[6] = mapToGripperJoints(45);
            jointState_msg_robot.data[7] = mapToGripperJoints(45);
            jointState_msg_robot.data[8] = mapToGripperJoints(45);
        }
        else
        {
            jointState_msg_robot.data[6] = mapToGripperJoints(100);
            jointState_msg_robot.data[7] = mapToGripperJoints(100);
            jointState_msg_robot.data[8] = mapToGripperJoints(100);
        }
    }

    pub_des_jstate.publish(jointState_msg_robot);
    loop_rate.sleep();
}

/**
 * @brief     This function is used to move the robot to accomplish the assignment 1 and 2
 * 
 */
void move()
{
    ros::Rate loop_rate(LOOP_RATE);
    float dt; // time step
    dt = 0.001;
    Vector3f target;
    target << pose.position(0), pose.position(1), .6;
    // go above the desired position
    invDiffKinematicControlSimComplete(target, pose.orientation, dt);
    // go to the desired position
    target << pose.position(0), pose.position(1), pose.position(2);
    invDiffKinematicControlSimComplete(target, pose.orientation, dt);
    // grasp
    grasp = 1;
    if (real_robot)
    {
        graspit();
    }
    else
    {
        sendJointState(TH0);
    }
    for (int i = 0; i < 50; i++)
    {
        loop_rate.sleep();
    }
    // middle set point 
    target << 0, -0.4, 0.6;
    invDiffKinematicControlSimComplete(target, pose.orientation, dt);
    // go to the desired position depending on the class of the block
    switch (class_id)
        {
        case 0:
            target << .4, 0, .82;
            break;

        case 1:
            target << .4, -.05, .82;
            break;

        case 2:
            target << .4, -.1, .82;
            break;

        case 3:
            target << .4, -.15, .82;
            break;

        case 4:
            target << .4, -.20, .82;
            break;

        case 5:
            target << .4, -.25, .82;
            break;

        case 6:
            target << .4, -.30, .82;
            break;

        case 7:
            target << .4, -.35, .82;
            break;

        case 8:
            target << .4, -.40, .82;
            break;

        case 9:
            target << .3, -.40, .82;
            break;

        case 10:
            target << .3, -.35, .82;
            break;

        default:
            break;
        }
    invDiffKinematicControlSimComplete(target, pose.orientation, dt);
    // ungrasp
    grasp = 0;
    if (real_robot)
    {
        graspit();
    }
    else
    {
        sendJointState(TH0);
    }
    for (int i = 0; i < 50; i++)
    {
        loop_rate.sleep();
    }
    // lift a little bit
    target(2) = 0.65;
    invDiffKinematicControlSimComplete(target, pose.orientation, dt);
    sendJointState(TH0);
    // go to the middle point
    target << 0, -0.4, 0.6;
    invDiffKinematicControlSimComplete(target, pose.orientation, dt);
    // go to the starting position
    startingPosition();
    ack();
}

/**
 * @brief      This function is used to go to the starting position
 */
void startingPosition()
{
    float dt; // time step
    dt = 0.001;
    Vector3f target;
    target << -.4, -.4, .6;
    invDiffKinematicControlSimComplete(target, pose.orientation, dt);
}

/**
 * @brief     This function is used to send the ack to the taskManager
 *              - send it when the robot has finished the motion task
 * 
 */
void ack()
{
    ros::Rate loop_rate(LOOP_RATE);
    std_msgs::Int32 ack;
    ack.data = 1;
    // wait a little bit before sending the ack to the taskManager (not stress too much the robot)
    for (int i = 0; i < 40; i++)
    {
        loop_rate.sleep();
    }
    ack_pos.publish(ack);
}

/**
 * @brief     This function is used to map the gripper joints to the real robot
 * 
 */
void graspit()
{
    ros_impedance_controller::generic_float gripper_diameter;
    if (grasp)
    {
        gripper_diameter.request.data = 60;
        client.call(gripper_diameter);
    }
    else
    {
        gripper_diameter.request.data = 100;
        client.call(gripper_diameter);
    }
}