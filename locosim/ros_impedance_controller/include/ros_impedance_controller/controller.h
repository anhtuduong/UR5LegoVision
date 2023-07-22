#ifndef IMP_CONTROLLER_H
#define IMP_CONTROLLER_H

// Ros
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ros_impedance_controller/EffortPid.h>
#include <ros_impedance_controller/get_effort_pid.h>
#include <ros_impedance_controller/pid.h>
#include <ros_impedance_controller/set_pids.h>
// PluginLib
#include <pluginlib/class_list_macros.hpp>
// Ros control
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
// Hardware interfaces
#include <hardware_interface/joint_command_interface.h>

//pid
#include <control_toolbox/pid.h>
// Eigen
#include <Eigen/Dense>

//map
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h> //robotname/groudtruth
#include <ros_impedance_controller/BaseState.h> //robotname/pose



namespace ros_impedance_controller
{

class Controller : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
{
public:
    /** @brief Constructor function */
    Controller();

    /** @brief Destructor function */
    ~Controller();

    /**
         * @brief Initializes sample controller
         * @param hardware_interface::RobotHW* robot hardware interface
         * @param ros::NodeHandle& Root node handle
         * @param ros::NodeHandle& Supervisor node handle
         */
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    /**
         * @brief Starts the sample controller when controller manager request it
         * @param const ros::Time& time Time
         */
    void starting(const ros::Time& time);

    /**
         * @brief Updates the sample controller according to the control
         * frequency (task frequency)
         * @param const ros::Time& time Time
         * @param const ros::Duration& Period
         */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
         * @brief Stops the sample controller when controller manager request it
         * @param const ros::time& Time
         */
    void stopping(const ros::Time& time);

private:



    void commandCallback(const sensor_msgs::JointState &msg);
    bool setPidsCallback(set_pids::Request& req,
                         set_pids::Response& res);
    void updateDiscreteGains(const std::vector<double> & joint_p_gain_, const std::vector<double> & joint_d_gain_,  const std::vector<double> & joint_i_gain_, const int & N, const double & Ts);
    //no longer used
    void baseGroundTruthCB(const nav_msgs::OdometryConstPtr &msg);
    

    ros::Subscriber command_sub_;
    //no longer used
    ros::Subscriber gt_sub_;
    ros::ServiceServer set_pids_srv_;
    ros::ServiceServer get_map_srv_;

    std::shared_ptr<realtime_tools::RealtimePublisher<BaseState>> pose_pub_rt_;   
    ros::Publisher effort_pid_pub;
    

    /** @brief Number of joints */
    unsigned int num_joints_;
    /** @brief Joint names */
    std::vector<std::string> joint_names_;
    /** @brief Joint states for reading positions, velocities and efforts and writing effort commands */
    std::vector<hardware_interface::JointHandle> joint_states_;
    /** @brief Desired joint efforts */
    Eigen::VectorXd des_joint_efforts_;
    /** @brief Desired joint positions */
    Eigen::VectorXd des_joint_positions_;
    /** @brief Old joint error */
    Eigen::VectorXd integral_action_old_;
    /** @brief Desired joint velocities */
    Eigen::VectorXd des_joint_velocities_;
    /** @brief Old joint error */
    Eigen::VectorXd proportional_action_;
    /** @brief Old joint error */
    Eigen::VectorXd integral_action_;
    /** @brief Old joint error */
    Eigen::VectorXd derivative_action_;
    /** @brief Actual P value for the joints PID controller */
    std::vector<double> joint_p_gain_;
    /** @brief Actual I value for the joints PID controller */
    std::vector<double> joint_i_gain_;
    /** @brief Actual D value for the joints PID controller */
    std::vector<double> joint_d_gain_;
    std::vector<std::string> joint_type_;
    Eigen::VectorXd measured_joint_position_;
    Eigen::VectorXd measured_joint_velocity_;

     Eigen::VectorXd joint_positions_old_;

    /** @brief Desired joint efforts computed by the PIDs */
    Eigen::VectorXd des_joint_efforts_pids_;
    

    /** @brief Discrete implem varables */
    Eigen::VectorXd error_;
    Eigen::VectorXd error1_;
    Eigen::VectorXd T_d, T_i;
    Eigen::VectorXd use_integral_action_;
    const int N = 8;
    double Ts = 0.;

    // no longer used
    //tf::Quaternion q_base;
    //tf::Vector3 base_pos_w;
    //geometry_msgs::Twist base_twist_w;

    std::vector<std::vector<double> > force_;
    std::vector<std::vector<double> > torque_;
    std::vector<std::vector<double> > normal_;

    ros::NodeHandle * root_nh_;
    bool verbose = false;
    bool discrete_implementation_ = false;

};


PLUGINLIB_EXPORT_CLASS(ros_impedance_controller::Controller, controller_interface::ControllerBase);

} //@namespace ros_impedance_controller

#endif //IMP_CONTROLLER_H
