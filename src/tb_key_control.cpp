#include "tb_key_control.hpp"

TbKeyController::TbKeyController() : Node("tb_key_controller_node")
{
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(4).reliable());

    tcgetattr(STDIN_FILENO, &old_tio_);         // Setting up a terminal to read keys without Enter
    
    new_tio_ = old_tio_;
    new_tio_.c_lflag &= ~(ICANON);

    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(KEYBOARD_CHECK_PERIOD),
        std::bind(&TbKeyController::TimerCallback_, this));

    RCLCPP_INFO(this->get_logger(), "Use WASD for robot control, Q for quit");
}

TbKeyController::~TbKeyController()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);    // Restoring terminal settings
}


void TbKeyController::TimerCallback_()
{
    geometry_msgs::msg::Twist tb_twist = geometry_msgs::msg::Twist();

    char pressed_key = 0;


    switch (pressed_key) 
    {
    case TB_MOVE_FORWARD_KEY: tb_twist.linear.x  =  TB_MOVE_SPEED;  break;
    case TB_MOVE_BACK_KEY   : tb_twist.linear.x  = -TB_MOVE_SPEED;  break;
    case TB_TURN_LEFT_KEY   : tb_twist.angular.z =  TB_TURN_SPEED;  break;
    case TB_TURN_RIGHT_KEY  : tb_twist.angular.z = -TB_TURN_SPEED;  break;

    default                 :  tb_twist.linear.x  = 0;
                               tb_twist.angular.z = 0;              break;
    }

    velocity_pub_->publish(tb_twist);
}

int main(const int argc, const char *argv[])
{
    rclcpp::init(argc, argv);

    auto tb_key_controller_node = std::make_shared<TbKeyController>();
    rclcpp::spin(tb_key_controller_node);

    rclcpp::shutdown();
}