#ifndef VELOCITY_ESTIMATOR_VELOCITYESTIMATOR_HPP
#define VELOCITY_ESTIMATOR_VELOCITYESTIMATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace examples
{

class VelocityEstimator : public rclcpp::Node
{
public:
    VelocityEstimator();

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_subPose_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   m_pubVelocity_;

    bool         m_havePrev{false};
    double       m_prevX{0.0};
    rclcpp::Time m_prevStamp{0, 0, RCL_ROS_TIME};
};

} // namespace examples

#endif // VELOCITY_ESTIMATOR_VELOCITYESTIMATOR_HPP
