#ifndef TB_KEY_CONTROL_HPP
#define TB_KEY_CONTROL_HPP

#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


const auto KEYBOARD_CHECK_PERIOD = std::chrono::milliseconds(50);

constexpr char TB_MOVE_FORWARD_KEY = 'w';
constexpr char TB_MOVE_BACK_KEY    = 's';
constexpr char TB_TURN_LEFT_KEY    = 'a';
constexpr char TB_TURN_RIGHT_KEY   = 'd';
constexpr char TB_STOP_KEY         = ' ';

constexpr char TB_KEY_CONTROLLER_QUIT_KEY = 'q';

constexpr double TB_MOVE_SPEED = 0.5;       // meters  per sec
constexpr double TB_TURN_SPEED = 0.5;       // radians per sec

class TbKeyController : public rclcpp::Node
{
public:
    TbKeyController();
    ~TbKeyController();

private:
    void TimerCallback_();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::TimerBase::SharedPtr                            timer_;

    termios old_tio_, new_tio_;
};

#endif // TB_KEY_CONTROL_HPP