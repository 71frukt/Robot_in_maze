#ifndef TB_MANUAL_CONTROL_HPP
#define TB_MANUAL_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <keyboard_msgs/msg/key.hpp>


class TbKeyController : public rclcpp::Node
{
public:
    TbKeyController();

    enum KeyType   { KEYUP, KEYDOWN };
    struct KeyInfo { char num; KeyType key_type; };
    
    
private:
    static constexpr const char *const KEYDOWN_TOPIC_NAME_ = "/keydown"; 
    static constexpr const char *const KEYUP_TOPIC_NAME_   = "/keyup"; 
    
    static constexpr char   TB_MOVE_FORWARD_KEY_  = 'w';
    static constexpr char   TB_MOVE_BACKWARD_KEY_ = 's';
    static constexpr char   TB_TURN_LEFT_KEY_     = 'a';
    static constexpr char   TB_TURN_RIGHT_KEY_    = 'd';

    static constexpr double TB_MOVE_SPEED         = 2;       // meters  per sec
    static constexpr double TB_TURN_SPEED         = 2;        // radians per sec

    static const size_t KEYS_NUM = 256;
    KeyInfo keys_info_[KEYS_NUM]  = {};
    
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keydown_subscr_;
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keyup_subscr_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    
    KeyInfo GetKeyInfo_(const char key_num)
    {
        return keys_info_[u_char(key_num)];
    }
    
    void KeyEventCallback_(const keyboard_msgs::msg::Key::SharedPtr msg, const KeyType key_event_type);

    void RobotKeyControl_();
};


#endif // TB_MANUAL_CONTROL_HPP