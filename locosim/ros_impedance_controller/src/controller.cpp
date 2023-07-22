/**
 * @file controller.cpp
 * @author Gennaro Raiola, Michele Focchi
 * @date 12 June, 2018
 * @brief Ros impedance controller.
 */

#include <ros_impedance_controller/controller.h>
#include<string.h>
#include <math.h>

namespace ros_impedance_controller {

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

Controller::Controller()
{
}

Controller::~Controller()
{
}
   
bool Controller::init(hardware_interface::RobotHW* robot_hw,
                      ros::NodeHandle& root_nh,
                      ros::NodeHandle& controller_nh)
{
    // getting the names of the joints from the ROS parameter server
    std::cout<< cyan<< "ROS_IMPEDANCE CONTROLLER: Initialize Ros Impedance Controller" << reset <<std::endl;
    root_nh_ = &root_nh;
    assert(robot_hw);

//TODO
//    std::string srdf, urdf;
//    if(!controller_nh.getParam("/robot_description",urdf))
//    {
//        ROS_ERROR_NAMED(CLASS_NAME,"robot_description not available in the ros param server");

//    }
//    if(!controller_nh.getParam("/robot_semantic_description",srdf))
//    {
//        ROS_ERROR_NAMED(CLASS_NAME,"robot_description_semantic not available in the ros param server");

//    }

    hardware_interface::EffortJointInterface* eff_hw = robot_hw->get<hardware_interface::EffortJointInterface>();

    if(!eff_hw)
    {
        ROS_ERROR("hardware_interface::EffortJointInterface not found");
        return false;
    }
    
    if (!controller_nh.getParam("joints", joint_names_))
    {
        ROS_ERROR("No joints given in the namespace: %s.", controller_nh.getNamespace().c_str());
        return false;
    } else
    {
         std::cout<< green<< "Found " <<joint_names_.size()<< " joints"<< reset <<std::endl;
    }

     // Setting up handles:
    for ( int i = 0; i < joint_names_.size(); i++)
    {

        // Getting joint state handle
        try
        {
            std::cout<< green<< "Loading effort interface for joint " <<joint_names_[i]<< reset <<std::endl;
            joint_states_.push_back(eff_hw->getHandle(joint_names_[i]));

        }
        catch(...)
        {
            ROS_ERROR("Error loading the effort interfaces");
            return false;
        }
    }
    assert(joint_states_.size()>0);

    //subscriber to the ground truth
    std::string robot_name;
    ros::NodeHandle param_node;
    param_node.getParam("/robot_name", robot_name);
    param_node.getParam("/pid_discrete_implementation", discrete_implementation_);
    if (discrete_implementation_)
    {
        std::cout<< green<< "Discrete implementation of PID control! (low the gains)"<< reset <<std::endl; 
    }
    else
    {
        std::cout<< green<< "Continuous implementation of PID control!"<< reset <<std::endl; 
    }


    // Resize the variables
    des_joint_positions_.resize(joint_states_.size());
    des_joint_positions_.fill(0.0);
    integral_action_old_.resize(joint_states_.size());
    integral_action_old_.fill(0.0);
    des_joint_velocities_.resize(joint_states_.size());
    des_joint_velocities_.fill(0.0);
    des_joint_efforts_.resize(joint_states_.size());
    des_joint_efforts_.fill(0.0);
    des_joint_efforts_pids_.resize(joint_states_.size());
    des_joint_efforts_pids_.fill(0.0);
    measured_joint_position_.resize(joint_states_.size());
    measured_joint_position_.fill(0.0);
    measured_joint_velocity_.resize(joint_states_.size());
    measured_joint_velocity_.fill(0.0);

    joint_positions_old_.resize(joint_states_.size());
    joint_positions_old_.resize(joint_states_.size());

    // discrete implementation 
    error_.resize(joint_states_.size());
    error_.fill(0.0);
    error1_.resize(joint_states_.size());
    error1_.fill(0.0);    

    T_i.resize(joint_states_.size());
    T_i.fill(0.0);
    T_d.resize(joint_states_.size());
    T_d.fill(0.0);    

    proportional_action_.resize(joint_states_.size());
    proportional_action_.fill(0.0); 
    integral_action_.resize(joint_states_.size());
    integral_action_.fill(0.0); 
    derivative_action_.resize(joint_states_.size());
    derivative_action_.fill(0.0); 

    use_integral_action_.resize(joint_states_.size());
    use_integral_action_.fill(false); 
    

    joint_type_.resize(joint_states_.size());
    std::fill(joint_type_.begin(), joint_type_.end(), "revolute");

    joint_p_gain_.resize(joint_states_.size());
    joint_i_gain_.resize(joint_states_.size());
    joint_d_gain_.resize(joint_states_.size());
    for (unsigned int i = 0; i < joint_states_.size(); i++)
    {
       // Getting PID gains
       if (!controller_nh.getParam("gains/" + joint_names_[i] + "/p", joint_p_gain_[i]))
       {
           ROS_ERROR("No P gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
           return false;
       }
       if (!controller_nh.getParam("gains/" + joint_names_[i] + "/i", joint_i_gain_[i]))
       {
           ROS_ERROR("No D gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
           return false;
       }
       if (!controller_nh.getParam("gains/" + joint_names_[i] + "/d", joint_d_gain_[i]))
       {
           ROS_ERROR("No I gain given in the namespace: %s. ", controller_nh.getNamespace().c_str());
           return false;
       }
       // Check if the values are positive
       if(joint_p_gain_[i]<0.0 || joint_i_gain_[i]<0.0 || joint_d_gain_[i]<0.0)
       {
           ROS_ERROR("PID gains must be positive!");
           return false;
       }
       ROS_DEBUG("P value for joint %i is: %f",i,joint_p_gain_[i]);
       ROS_DEBUG("I value for joint %i is: %f",i,joint_i_gain_[i]);
       ROS_DEBUG("D value for joint %i is: %f",i,joint_d_gain_[i]);

       controller_nh.getParam("joint_type/" + joint_names_[i], joint_type_[i]);

       //get statrup go0 position from yaml (TODO srdf)
       controller_nh.getParam("home/" + joint_names_[i], des_joint_positions_[i]);
     
        if (joint_i_gain_[i] == 0)
        {
            use_integral_action_[i] = false;
            T_i[i] = 0;
        }
        else
        {
            use_integral_action_[i] = true;
            T_i[i] = joint_p_gain_[i] / joint_i_gain_[i];
        }


        if (joint_p_gain_[i] == 0)
        {
            T_d[i] = 0;
        }
        else
        {
            T_d[i] = joint_d_gain_[i] / joint_p_gain_[i];
        }
    }


    // Create the subscriber
    command_sub_ = root_nh.subscribe("/command", 1, &Controller::commandCallback, this, ros::TransportHints().tcpNoDelay());

  


    std::cout<< cyan<< "ROS_IMPEDANCE CONTROLLER: ROBOT NAME IS : "<< robot_name<<reset <<std::endl;
     // Create the PID set service
    set_pids_srv_ = param_node.advertiseService("/set_pids", &Controller::setPidsCallback, this);

    effort_pid_pub = root_nh.advertise<EffortPid>("effort_pid", 1);
    return true;
}


void Controller::starting(const ros::Time& time)
{
    ROS_DEBUG("Starting Controller");

}
                           
                                 


bool Controller::setPidsCallback(set_pids::Request& req,
                                 set_pids::Response& res)
{
    //get params from parameter server
    root_nh_->getParam("/verbose", verbose);
    res.ack = true;

    for(unsigned int i = 0; i < req.data.size(); i++)
    {
        for(unsigned int j = 0; j < joint_names_.size(); j++)
            if(!std::strcmp(joint_names_[j].c_str(),req.data[i].joint_name.c_str()))
            {
                if(req.data[i].p_value>=0.0)
                {
                    joint_p_gain_[j] = req.data[i].p_value;
                    if (verbose)
                        std::cout<<"Set P gain for joint "<< joint_names_[j] << " to: "<<joint_p_gain_[j]<<std::endl;
                }
                else
                {
                   ROS_WARN("P value has to be positive");
                   res.ack = false;
                }

                if(req.data[i].i_value>=0.0)
                {
                    joint_i_gain_[j] = req.data[i].i_value;
                    if (verbose)
                       std::cout<<"Set I gain for joint "<< joint_names_[j] << " to: "<<joint_i_gain_[j]<<std::endl;
                }
                else
                {
                   ROS_WARN("I value has to be positive");
                   res.ack = false;
                }

                if(req.data[i].d_value>=0.0)
                {
                    joint_d_gain_[j] = req.data[i].d_value;
                    if (verbose)
                       std::cout<<"Set D gain for joint "<< joint_names_[j] << " to: "<<joint_d_gain_[j]<<std::endl;
                }
                else
                {
                   ROS_WARN("D value has to be positive");
                   res.ack = false;
                }

                if (joint_i_gain_[i] == 0)
                {
                    use_integral_action_[j] = false;
                    T_i[j] = 0;
                }
                else
                {
                    use_integral_action_[j] = true;
                    T_i[j] = joint_p_gain_[j] / joint_i_gain_[j];
                }

                if (joint_p_gain_[i] == 0)
                {
                    T_d[i] = 0;
                }
                else
                {
                    T_d[i] = joint_d_gain_[i] / joint_p_gain_[i];
                }
                

                }


    }

    return true;
}

void Controller::commandCallback(const sensor_msgs::JointState& msg)
{

    if(joint_states_.size() == msg.position.size() && joint_states_.size() == msg.velocity.size() && joint_states_.size() == msg.effort.size())
    {
            //des_joint_efforts_(i) = msg.data[i];
            des_joint_positions_ = Eigen::Map<const Eigen::VectorXd>(&msg.position[0],joint_states_.size());
            des_joint_velocities_ = Eigen::Map<const Eigen::VectorXd>(&msg.velocity[0],joint_states_.size());
            des_joint_efforts_ = Eigen::Map<const Eigen::VectorXd>(&msg.effort[0],joint_states_.size());
    }

    else
        ROS_WARN("Wrong dimension!");
}


void Controller::baseGroundTruthCB(const nav_msgs::OdometryConstPtr &msg)
{

    //static tf::TransformBroadcaster br;
    //tf::Transform w_transform_b;

    //orientation of base frame
/*
    q_base.setX(msg->pose.pose.orientation.x);
    q_base.setY(msg->pose.pose.orientation.y);
    q_base.setZ(msg->pose.pose.orientation.z);
    q_base.setW(msg->pose.pose.orientation.w);
    //position of base frame
    base_pos_w = tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
*/
    //get twist
/*
    base_twist_w.linear.x= msg->twist.twist.linear.x;
    base_twist_w.linear.y = msg->twist.twist.linear.y;
    base_twist_w.linear.z = msg->twist.twist.linear.z;
    base_twist_w.angular.x = msg->twist.twist.angular.x;
    base_twist_w.angular.y = msg->twist.twist.angular.y;
    base_twist_w.angular.z = msg->twist.twist.angular.z;
*/

    //the vector of the base is in the world frame, so to apply to the base frame I should rotate it to the base frame before
    //tf::Vector3 world_origin_w(-msg->pose.pose.position.x,-msg->pose.pose.position.y,-msg->pose.pose.position.z);
    //tf::Vector3 world_origin_b = tf::quatRotate(q_base.inverse(), world_origin_w);

    //this is the transform from base to world to publish the world transform for rviz
    //w_transform_b.setRotation(q_base.inverse());
    //w_transform_b.setOrigin(world_origin_b);
    //br.sendTransform(tf::StampedTransform(w_transform_b, ros::Time::now(), "/base_link", "/world" ));
}



void Controller::update(const ros::Time& time, const ros::Duration& period)
{

    //if task_period is smaller than sim max_step_size (in world file) period it is clamped to that value!!!!!
    //std::cout<<period.toSec()<<std::endl;
//    std::cout<<"des_joint_efforts_: " << des_joint_efforts_.transpose()<<std::endl;
//    std::cout<<"des_joint_velocities_: " << des_joint_velocities_.transpose()<<std::endl;
//    std::cout<<"des_joint_positions_: " << des_joint_positions_.transpose()<<std::endl;
    // Write to the hardware interface
    //(NB this is not the convention of ros but the convention that we define in ros_impedance_controller_XX.yaml!!!!
    //std::cout << "-----------------------------------" << std::endl;
    

    
    EffortPid msg;
    msg.name.resize(joint_states_.size());
    msg.effort_pid.resize(joint_states_.size());

    if (discrete_implementation_)
    {
        Ts = period.toSec();
        // check: http://www.diag.uniroma1.it/deluca/automation/Automazione_RegolazionePID.pdf
        for (unsigned int i = 0; i < joint_states_.size(); i++)
        {
            //discrete implementation
            error1_[i] = error_[i];
            error_[i] = des_joint_positions_(i) - joint_states_[i].getPosition();
            

            proportional_action_[i] = joint_p_gain_[i] * error_[i];
            if (use_integral_action_[i])
            {
                integral_action_[i] = integral_action_[i] + (joint_p_gain_[i] * Ts / T_i[i]) * error_[i];
            }
            derivative_action_[i] = 1/(1+T_d[i]/(N*Ts)) * (T_d[i]/(N*Ts)*derivative_action_[i] + joint_p_gain_[i]*T_d[i]/Ts * (error_[i]-error1_[i]));
            

            des_joint_efforts_pids_(i) = proportional_action_[i] + integral_action_[i] + derivative_action_[i];
            
            msg.name[i] = joint_names_[i];
            msg.effort_pid[i]=des_joint_efforts_pids_(i);
            //add PID + FFWD
            joint_states_[i].setCommand(des_joint_efforts_(i) +  des_joint_efforts_pids_(i));

            joint_positions_old_[i] = joint_states_[i].getPosition();
        }


    } else{
        for (unsigned int i = 0; i < joint_states_.size(); i++)
        {      
        
            measured_joint_position_(i) = joint_states_[i].getPosition();
            measured_joint_velocity_(i) = joint_states_[i].getVelocity();
            double joint_pos_error = des_joint_positions_(i) - measured_joint_position_(i);
            double integral_action = integral_action_old_[i] + joint_i_gain_[i]*joint_pos_error*period.toSec();
            //std::cout << "***** joint: "<< joint_names_[i] << std::endl;
            //std::cout << "joint des:   "<< des_joint_positions_(i) << std::endl;
            //std::cout << "joint pos:   "<< joint_states_[i].getPosition() << std::endl;
            //std::cout << "wrap:        "<< measured_joint_position_(i) << std::endl;
        
            //std::cout << "effort pid des:  "<< des_joint_efforts_pids_(i) << std::endl;
            //std::cout << "effort meas: "<< joint_states_[i].getEffort() << std::endl;
        
            //compute PID
            des_joint_efforts_pids_(i) = joint_p_gain_[i]*(des_joint_positions_(i) -  measured_joint_position_(i) ) +
                                        joint_d_gain_[i]*(des_joint_velocities_(i) - measured_joint_velocity_(i) ) +
                                        integral_action;

            integral_action_old_[i] = integral_action;

            msg.name[i] = joint_names_[i];
            msg.effort_pid[i]=des_joint_efforts_pids_(i);
            //add PID + FFWD
            joint_states_[i].setCommand(des_joint_efforts_(i) +  des_joint_efforts_pids_(i));
            
        }
    }



    effort_pid_pub.publish(msg);
}



void Controller::stopping(const ros::Time& time)
{
    ROS_DEBUG("Stopping Controller");
}

} //namespace
