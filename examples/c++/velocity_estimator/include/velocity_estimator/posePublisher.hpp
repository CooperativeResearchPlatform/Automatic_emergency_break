#ifndef POSE_PUBLISHER_VELOCITYESTIMATOR_HPP
#define POSE_PUBLISHER_VELOCITYESTIMATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace examples
{

class PosePublisher : public rclcpp::Node
{
public:
    PosePublisher();

private:
    void timerCallback();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pubPose_;

    rclcpp::TimerBase::SharedPtr m_timer_;

    double  m_x{0.0};
    double  m_t{0.0};
    float   m_dt{0.1f};
};

} // namespace examples

#endif // POSE_PUBLISHER_VELOCITYESTIMATOR_HPP
