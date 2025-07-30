#include <rclcpp/rclcpp.hpp>
#include <keyboard_msgs/msg/key.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "autonomous_robot/tb_key_control.hpp"


TbKeyController::TbKeyController() : Node("key_listener")
{
    // subscribe on keydown
    keydown_subscr_ = create_subscription<keyboard_msgs::msg::Key>(
        KEYDOWN_TOPIC_NAME_,
        10,

        [this](const keyboard_msgs::msg::Key::SharedPtr msg) {
            KeyEventCallback_(msg, TbKeyController::KEYDOWN);
        }

        // std::bind(&TbKeyController::KeyEventCallback_, this, std::placeholders::_1, TbKeyController::KEYDOWN)
    );

    keyup_subscr_ = create_subscription<keyboard_msgs::msg::Key>(
        KEYUP_TOPIC_NAME_,
        10,

        [this](const keyboard_msgs::msg::Key::SharedPtr msg) {
            KeyEventCallback_(msg, TbKeyController::KEYUP);
        }
        
        // std::bind(&TbKeyController::KeyEventCallback_, this, std::placeholders::_1, TbKeyController::KEYUP)
    );
    


    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(4).reliable());
}

void TbKeyController::KeyEventCallback_(const keyboard_msgs::msg::Key::SharedPtr key_msg, const KeyType key_type)
{
    keys_info_[key_msg->code].key_type = key_type;

    RobotKeyControl_();
}

void TbKeyController::RobotKeyControl_()
{
    geometry_msgs::msg::Twist tb_twist = geometry_msgs::msg::Twist();

    if (GetKeyInfo_(TB_MOVE_FORWARD_KEY_).key_type == KEYDOWN)
        tb_twist.linear.x = TB_MOVE_SPEED;

    else if (GetKeyInfo_(TB_MOVE_BACKWARD_KEY_).key_type == KEYDOWN)
        tb_twist.linear.x = -TB_MOVE_SPEED;

    else
        tb_twist.linear.x = 0;


    if (GetKeyInfo_(TB_TURN_LEFT_KEY_).key_type == KEYDOWN)
        tb_twist.angular.z = TB_TURN_SPEED;

    else if (GetKeyInfo_(TB_TURN_RIGHT_KEY_).key_type == KEYDOWN)
        tb_twist.angular.z = -TB_TURN_SPEED;

    else
        tb_twist.angular.z = 0;

    velocity_pub_->publish(tb_twist);
}

int main(const int argc, const char *argv[])
{
    rclcpp::init(argc, argv);

    auto tb_key_controller_node = std::make_shared<TbKeyController>();
    rclcpp::spin(tb_key_controller_node);

    rclcpp::shutdown();
}