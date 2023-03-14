/**
 * @file taskManager.cpp
 * @author De Martini Davide (davide.demartini@studenti.unitn.it)
 * @brief This file contains the task manager node
 * @version 1
 * @date 2023-02-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ros/ros.h"
#include "kinematics.h"
#include "motion/pos.h"
#include <std_msgs/Int32.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <complex>

// ------------------- DEFINES ------------------- //

/// @brief Loop rate of the node
#define LOOP_RATE 1000
/// @brief Flag to indicate if the robot is real or simulated
#define REAL_ROBOT 0

// ------------------- NAMESPACES ------------------- //

using namespace std;
using namespace Eigen;

// ------------------- GLOBAL VARIABLES ------------------- //

/// @brief Publisher for the position of the block
ros::Publisher pub_pos;
/// @brief Publisher for the ack
ros::Publisher pub_ack;
/// @brief Subscriber for the vision topic
ros::Subscriber sub_vision;
/// @brief Subscriber for the ack topic
ros::Subscriber sub_ack;
/// @brief Subscriber for the stop topic
ros::Subscriber sub_stop;
/// @brief Flag to indicate that the vision msg is received
int vision_received = 0;
/// @brief Flag to indicate that the vision is on
int vision_on = 1;
/// @brief Position of the block
Vector3f block_pos;
/// @brief Rotation of the block
Vector3f block_rot;
/// @brief Class of the block
int block_class;
/// @brief Flag to indicate that the motion is finished
int ready = 1;
/// @brief Flag to indicate that the task manager has to stop
int stop = 0;

float Z = 0.835;

// ------------------- FUNCTIONS PROTOTIPES ------------------- //

Vector3f worldToBase(Vector3f xw);
Vector3f cameraToWorld(Vector3f xw);
void visionCallback(const motion::pos::ConstPtr &msg);
void ackCallback(const std_msgs::Int32::ConstPtr &msg);
void stopCallback(const std_msgs::Int32::ConstPtr &msg);
void ack();

// ------------------- MAIN ------------------- //

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle n;

    pub_pos = n.advertise<motion::pos>("/motion/pos", 1);
    pub_ack = n.advertise<std_msgs::Int32>("/vision/ack", 1);

    sub_vision = n.subscribe("/vision/pos", 1, visionCallback);
    sub_ack = n.subscribe("/motion/ack", 1, ackCallback);
    sub_stop = n.subscribe("/taskManager/stop", 1, stopCallback);

    ros::Rate loop_rate(LOOP_RATE);

    motion::pos msg;

    while (ros::ok())
    {
        while (pub_pos.getNumSubscribers() < 1)
            loop_rate.sleep();
        if (!vision_on)             // If the vision is not on, the user has to insert the position of the block        
        {
            if (ready)              // If the motion is finished, the user can insert the position of the block
            {
                ready = 0;
                cout << "Insert the position (x y z roll pitch yaw): ";
                cin >> msg.x >> msg.y >> msg.z >> msg.roll >> msg.pitch >> msg.yaw;
                cout << "Insert the class id: ";
                cin >> msg.class_id;
                pub_pos.publish(msg);
            }
        }
        else                                      // Vision is on
        {
            if (vision_received && ready && !stop)         // If vision msg is received and the motion is finished, send the position to the motion
            {
                ready = 0;
                cout << "Blocks coordinates received from vision" << endl;
                cout << "Block position: " << block_pos.transpose() << endl;
                cout << "Block rotation: " << block_rot.transpose() << endl;
                cout << "Block class: " << block_class << endl;
                if (REAL_ROBOT)
                    block_pos = cameraToWorld(block_pos);
                else
                block_pos = worldToBase(block_pos);
                msg.x = block_pos(0);
                msg.y = block_pos(1);
                msg.z = Z;
                msg.roll = block_rot(0);
                msg.pitch = block_rot(1);
                msg.yaw = block_rot(2);
                msg.class_id = block_class;
                vision_received = 0;
                pub_pos.publish(msg);
            }
        }
        ros::spinOnce();
    }
}

// ------------------- FUNCTIONS DEFINITIONS ------------------- //

/**
 * @brief Convert the position of the block from the world frame to the base frame
 * 
 * @param xw 3D vector representing the position of the block in the world frame
 * @return Vector3f 
 */
Vector3f worldToBase(Vector3f xw)
{
    Matrix4f T;
    Vector3f xb;
    Vector4f xt;
    T << 1, 0, 0, 0.5,
        0, -1, 0, 0.35,
        0, 0, -1, 1.75,
        0, 0, 0, 1;
    xt = T.inverse() * Vector4f(xw(0), xw(1), xw(2), 1);
    xb << xt(0), xt(1), xt(2);
    return xb;
}

/**
 * @brief Convert the position of the block from the camera frame (real robot) to the world frame
 * 
 * @param xw 3D vector representing the position of the block in the camera frame
 * @return Vector3f 
 */
Vector3f cameraToWorld(Vector3f xw)
{
    Matrix4f T;
    Vector3f xb;
    Vector4f xt;
    T << 0.866, 0, 0.5, -0.4,
              0, 1, 0, 0.53,
              -0.5, 0, 0.866, 1.4,
              0, 0, 0, 1;
    xt = T * Vector4f(xw(0), xw(1), xw(2), 1);
    xb << xt(0), xt(1), xt(2);
    return xb;
}

/**
 * @brief Callback function for the vision topic, sets the detected position of the block
 * 
 * @param msg message received
 */
void visionCallback(const motion::pos::ConstPtr &msg)
{
    vision_received = 1;                            // flag to indicate that the vision msg is received
    block_pos << msg->x, msg->y, msg->z;
    block_rot << msg->roll, msg->pitch, msg->yaw;
    block_class = msg->class_id;
}

/**
 * @brief Callback function for the ack topic, sets the ready variable -> finished the motion
 * 
 * @param msg message received
 */
void ackCallback(const std_msgs::Int32::ConstPtr &msg)
{
    ready = msg->data;
    ack();
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
    pub_ack.publish(ack);
}

/**
 * @brief Callback function for the stop topic, sets the stop variable -> stop the motion -> finished the task
 * 
 * @param msg 
 */
void stopCallback(const std_msgs::Int32::ConstPtr &msg)
{
    stop = msg->data;
}