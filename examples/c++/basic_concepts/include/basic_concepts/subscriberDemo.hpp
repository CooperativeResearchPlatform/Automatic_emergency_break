#ifndef SUBSCRIBER_DEMO_BASICCONCEPTS_HPP
#define SUBSCRIBER_DEMO_BASICCONCEPTS_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace examples
{

class SubscriberDemo : public rclcpp::Node
{
public:
    SubscriberDemo();

private:
    void counterCallback(const std_msgs::msg::UInt8::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_subCounter_;
};

} // namespace examples

#endif // SUBSCRIBER_DEMO_BASICCONCEPTS_HPP