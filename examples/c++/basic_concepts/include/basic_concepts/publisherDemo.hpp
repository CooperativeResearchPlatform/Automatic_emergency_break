#ifndef PUBLISHER_DEMO_BASICCONCEPTS_HPP
#define PUBLISHER_DEMO_BASICCONCEPTS_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace examples
{

class PublisherDemo : public rclcpp::Node
{
public:
    PublisherDemo();

private:
    void timerCallback();

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_pubCounter_;

    rclcpp::TimerBase::SharedPtr m_timer_;

    uint8_t m_counter{0U};
};

} // namespace examples

#endif // PUBLISHER_DEMO_BASICCONCEPTS_HPP