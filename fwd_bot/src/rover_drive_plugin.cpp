#include "fwd_bot/rover_drive_plugin.hpp"

namespace gazebo
{

RoverDrivePlugin::RoverDrivePlugin() {}

RoverDrivePlugin::~RoverDrivePlugin()
{
    if (update_connection_)
    {
        update_connection_.reset();
    }
}

void RoverDrivePlugin::Load(physics::ModelPtr model, sdf::ElementPtr rover_sdf)
{
    model_ = model;
    std::string model_name = model_->GetName();
    RCLCPP_INFO(rclcpp::get_logger(RoverDrivePlugin::PLUGIN_NAME), "[%s] Model plugin '%s' was loaded", 
                                                                   RoverDrivePlugin::PLUGIN_NAME.c_str(), model_name.c_str());

    ros_node_ = gazebo_ros::Node::Get(rover_sdf);
    
    if (!ros_node_) 
    {
        RCLCPP_FATAL(rclcpp::get_logger(RoverDrivePlugin::PLUGIN_NAME), "[%s] Failed to get ROS node",
        RoverDrivePlugin::PLUGIN_NAME.c_str());
        return;
    }
    
    RCLCPP_INFO(ros_node_->get_logger(), "[%s] ROS 2 node of rover is initialized",
                                         RoverDrivePlugin::PLUGIN_NAME.c_str());
    
                                         // read params from SDF
    if (rover_sdf->HasElement(WHEEL_SEPARATION_SDF_NAME))                                             // TODO: in func with template
        wheel_separation_ = rover_sdf->Get<double>(WHEEL_SEPARATION_SDF_NAME);
    else
        RCLCPP_WARN(ros_node_->get_logger(), "[%s] Parameter '%s' is not defined, the default value is used: %.2f м", 
                                             RoverDrivePlugin::PLUGIN_NAME.c_str(), WHEEL_SEPARATION_SDF_NAME.c_str(), wheel_separation_);

    if (rover_sdf->HasElement(WHEEL_RADIUS_SDF_NAME))
        wheel_radius_ = rover_sdf->Get<double>(WHEEL_RADIUS_SDF_NAME);
    else
        RCLCPP_WARN(ros_node_->get_logger(), "[%s] Parameter '%s' is not defined, the default value is used: %.2f м",
                                             RoverDrivePlugin::PLUGIN_NAME.c_str(), WHEEL_RADIUS_SDF_NAME.c_str(), wheel_radius_);

    // read joints names
    ReadBoardWheelsJoints_(rover_sdf, LEFT_WHEELS_JOINTS_SDF_NAME,  left_wheels_joints_ );
    ReadBoardWheelsJoints_(rover_sdf, RIGHT_WHEELS_JOINTS_SDF_NAME, right_wheels_joints_);

    // subscription on /cmd_vel
    cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::QoS(10),
        std::bind(&RoverDrivePlugin::CmdVelCallback_, this, std::placeholders::_1)
    );

    RCLCPP_INFO(ros_node_->get_logger(), "Subscription on /cmd_vel is installed");

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RoverDrivePlugin::OnUpdate, this)
    );

    last_update_time_ = model_->GetWorld()->SimTime();

    RCLCPP_INFO(rclcpp::get_logger("gazebo"), "[%s] Plugin loaded for model %s",
                                              RoverDrivePlugin::PLUGIN_NAME.c_str(), model_->GetName().c_str());
}

void RoverDrivePlugin::ReadBoardWheelsJoints_(const sdf::ElementPtr rover_sdf, const std::string &joints_sdf_name, std::vector<physics::JointPtr> &target_joints)
{
    if (!rover_sdf->HasElement(joints_sdf_name))
    {
        RCLCPP_FATAL(ros_node_->get_logger(), "[%s] SDF tag '<%s>' is missing. Check your SDF.",
                                              RoverDrivePlugin::PLUGIN_NAME.c_str(), joints_sdf_name.c_str());
        return;
    }

    std::string joints_names_str = rover_sdf->GetElement(joints_sdf_name)->Get<std::string>();
    std::istringstream iss_left(joints_names_str);

    std::vector<std::string> joints_names;
    std::string token;
    
    while (iss_left >> token)
    {
        joints_names.push_back(token);
    }
    
    if (joints_names.empty())
    {
        RCLCPP_FATAL(ros_node_->get_logger(),
            "[%s] No joint names found in tag '%s' (empty or whitespace)",
            RoverDrivePlugin::PLUGIN_NAME.c_str(), joints_sdf_name.c_str());
        return;
    }

    for (const auto& joint_name : joints_names)
    {
        physics::JointPtr joint = model_->GetJoint(joint_name);

        if (!joint)
        {
            RCLCPP_FATAL(ros_node_->get_logger(), "[%s] Joint '%s' (declared in tag '<%s>') was not found", 
                                                  RoverDrivePlugin::PLUGIN_NAME.c_str(), joint_name.c_str(), joints_sdf_name.c_str());
            return;
        }

        target_joints.push_back(joint);
        RCLCPP_INFO(ros_node_->get_logger(), "[%s] Joint %s was found", 
                                             RoverDrivePlugin::PLUGIN_NAME.c_str(), joint_name.c_str());
    }
}

void RoverDrivePlugin::CmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;

    double left_velocity  = linear_velocity - angular_velocity * wheel_separation_ / 2.0;
    double right_velocity = linear_velocity + angular_velocity * wheel_separation_ / 2.0;

    target_left_wheels_ang_vel_  = left_velocity / wheel_radius_;
    target_right_wheels_ang_vel_ = right_velocity / wheel_radius_;

    RCLCPP_DEBUG(ros_node_->get_logger(), "[%s] Received: linear.x=%.2f, angular.z=%.2f → left_ω=%.2f, right_ω=%.2f",
                                          RoverDrivePlugin::PLUGIN_NAME.c_str(), linear_velocity, angular_velocity, target_left_wheels_ang_vel_, target_right_wheels_ang_vel_);

}

void RoverDrivePlugin::OnUpdate()
{
    common::Time current_time = model_->GetWorld()->SimTime();
    double dt = (current_time - last_update_time_).Double();

    if (dt <= 0.0) return;

    last_update_time_ = current_time;

    // set target velocity

    for (auto& joint : left_wheels_joints_)
        joint->SetVelocity(0, target_left_wheels_ang_vel_);

    for (auto& joint : right_wheels_joints_)
        joint->SetVelocity(0, target_right_wheels_ang_vel_);
}

GZ_REGISTER_MODEL_PLUGIN(RoverDrivePlugin);

}