#ifndef _ROVER_DRIVE_PLUGIN_HPP
#define _ROVER_DRIVE_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_ros/node.hpp>
// #include <gazebo/common/Plugin.hh>
// #include <gazebo/common/Events.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
// #include <memory>


// names of fields in sdf model
const std::string WHEEL_SEPARATION_SDF_NAME    = "wheel_separation"   ;    // TODO: put in class
const std::string WHEEL_RADIUS_SDF_NAME        = "wheel_radius"       ;
const std::string LEFT_WHEELS_JOINTS_SDF_NAME  = "left_wheels_joints" ;
const std::string RIGHT_WHEELS_JOINTS_SDF_NAME = "right_wheels_joints";


namespace gazebo
{

class RoverDrivePlugin : public ModelPlugin
{
public:
    inline static const std::string PLUGIN_NAME = "RoverDrivePlugin";

    RoverDrivePlugin();
    virtual ~RoverDrivePlugin();

    void Load(physics::ModelPtr model, sdf::ElementPtr rover_sdf) override; // is called by Gazebo on start
    void OnUpdate();                                                        // is called on each simulation tick

private:
    physics::ModelPtr model_;

    std::vector<physics::JointPtr> left_wheels_joints_;
    std::vector<physics::JointPtr> right_wheels_joints_;

    event::ConnectionPtr update_connection_;

    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    
    double target_left_wheels_ang_vel_  = 0.0;  // rad/sec
    double target_right_wheels_ang_vel_ = 0.0;

    double wheel_radius_     = 0.1;             // meters
    double wheel_separation_ = 0.5;

    common::Time last_update_time_;

    void CmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg);

    // reads from SDF names of joints in tag named joints_sdf_name and puts this joints in target_joints
    void ReadBoardWheelsJoints_(const sdf::ElementPtr sdf, const std::string &joints_sdf_name, std::vector<physics::JointPtr> &target_joints);
};

}   // namespace gazebo

#endif // _ROVER_DRIVE_PLUGIN_HPP