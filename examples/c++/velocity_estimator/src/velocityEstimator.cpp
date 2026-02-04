#include "velocity_estimator/velocityEstimator.hpp"


examples::VelocityEstimator::VelocityEstimator() : Node("velocity_estimator_demo")
{
    this->declare_parameter<std::string>("input_topic", "pose");
    this->declare_parameter<std::string>("output_topic", "velocity");
    this->declare_parameter<bool>("debug_enabled", false);

    std::string inputTopic;
    std::string outputTopic;

    this->get_parameter<std::string>("input_topic", inputTopic);
    this->get_parameter<std::string>("output_topic", outputTopic);

    m_pubVelocity_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        outputTopic,
        1
    );

    m_subPose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        inputTopic,
        1,
        std::bind(&VelocityEstimator::poseCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "velocity_estimator_demo node has been started");
}


void examples::VelocityEstimator::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    const rclcpp::Time stamp(msg->header.stamp);
    const double x = msg->pose.position.x;

    if (!m_havePrev)
    {
        m_prevX = x;
        m_prevStamp = stamp;
        m_havePrev = true;

        bool isDebugEnabled;
        this->get_parameter<bool>("debug_enabled", isDebugEnabled);

        if (isDebugEnabled)
            RCLCPP_INFO(this->get_logger(), "Received first pose - waiting for next message to compute derivative.");
        return;
    }

    const double dt = (stamp - m_prevStamp).seconds();

    if (dt < 0.0001)
    {
        bool isDebugEnabled;
        this->get_parameter<bool>("debug_enabled", isDebugEnabled);

        if (isDebugEnabled)
            RCLCPP_WARN(this->get_logger(), "dt too small (%.6f s) -> skipping velocity estimate", dt);
        return;
    }

    const double vx = (x - m_prevX) / dt;

    geometry_msgs::msg::TwistStamped velMsg;
    velMsg.header.stamp = stamp;
    velMsg.header.frame_id = msg->header.frame_id;

    velMsg.twist.linear.x = vx;
    velMsg.twist.linear.y = 0.0;
    velMsg.twist.linear.z = 0.0;

    velMsg.twist.angular.x = 0.0;
    velMsg.twist.angular.y = 0.0;
    velMsg.twist.angular.z = 0.0;

    m_pubVelocity_->publish(velMsg);

    bool isDebugEnabled;
    this->get_parameter<bool>("debug_enabled", isDebugEnabled);

    if (isDebugEnabled)
        RCLCPP_INFO(this->get_logger(), "Estimated velocity: vx=%.3f m/s (dt=%.3f s)", vx, dt);

    m_prevX = x;
    m_prevStamp = stamp;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<examples::VelocityEstimator>());
    rclcpp::shutdown();
    return 0;
}
