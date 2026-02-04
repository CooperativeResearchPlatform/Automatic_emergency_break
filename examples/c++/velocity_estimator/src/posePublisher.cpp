#include "velocity_estimator/posePublisher.hpp"


examples::PosePublisher::PosePublisher() : Node("pose_publisher")
{
    // parameters
    this->declare_parameter<std::string>("topic_name", "pose");

    this->declare_parameter<uint8_t>("publish_rate", 10);
    this->declare_parameter<bool>("debug_enabled", false);

    this->declare_parameter<double>("base_speed", 0.6);
    this->declare_parameter<double>("speed_amplitude", 0.4);
    this->declare_parameter<double>("speed_frequency", 0.15);

    std::string topicName;
    uint8_t publishRate;
    this->get_parameter<std::string>("topic_name", topicName);
    this->get_parameter<uint8_t>("publish_rate", publishRate);

    m_dt = 1.0f / static_cast<float>(publishRate);

    m_pubPose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        topicName,
        1
    );

    m_timer_ = this->create_wall_timer(
        std::chrono::duration<float>(1.0f / static_cast<float>(publishRate)),
        std::bind(&PosePublisher::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "pose_publisher_demo node has been started");
}


void examples::PosePublisher::timerCallback()
{
    double baseSpeed;
    double speedAmp;
    double speedFreq;
    this->get_parameter<double>("base_speed", baseSpeed);
    this->get_parameter<double>("speed_amplitude", speedAmp);
    this->get_parameter<double>("speed_frequency", speedFreq);

    const double v = baseSpeed + speedAmp * std::sin(2 * M_PI * speedFreq * m_t);

    m_x += v * m_dt;
    m_t += m_dt;

    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header.stamp = this->now();

    poseMsg.pose.position.x = m_x;
    poseMsg.pose.position.y = 0.0;
    poseMsg.pose.position.z = 0.0;

    poseMsg.pose.orientation.x = 0.0;
    poseMsg.pose.orientation.y = 0.0;
    poseMsg.pose.orientation.z = 0.0;
    poseMsg.pose.orientation.w = 1.0;

    m_pubPose_->publish(poseMsg);

    bool isDebugEnabled;
    this->get_parameter<bool>("debug_enabled", isDebugEnabled);

    if (isDebugEnabled)
        RCLCPP_INFO(this->get_logger(), "Published pose: x=%.3f m, v=%.3f m/s", m_x, v);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<examples::PosePublisher>());
    rclcpp::shutdown();
    return 0;
}
