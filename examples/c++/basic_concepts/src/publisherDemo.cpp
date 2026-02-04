#include "basic_concepts/publisherDemo.hpp"


examples::PublisherDemo::PublisherDemo() : Node("publisher_demo")
{
    // declare parameters
    this->declare_parameter<std::string>("topic_name", "counter");
    this->declare_parameter<uint8_t>("publish_rate", 5);
    this->declare_parameter<bool>("debug_enabled", false);

    std::string topicName;
    uint8_t     publishRate;

    // retrieve parameters
    this->get_parameter<std::string>("topic_name", topicName);
    this->get_parameter<uint8_t>("publish_rate", publishRate);

    // create the publisher
    m_pubCounter_ = this->create_publisher<std_msgs::msg::UInt8>(
        topicName,
        1
    );

    // init timer for publishing a counter
    m_timer_ = this->create_wall_timer(
        std::chrono::duration<float>(1.0f / static_cast<float>(publishRate)),
        std::bind(&PublisherDemo::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "publisher_demo node has been started");
}


void examples::PublisherDemo::timerCallback()
{
    // increment counter with and reset at 16
    m_counter = (m_counter+1) & 0xF;

    // create message and init with the counter value
    std_msgs::msg::UInt8 counterMsg;
    counterMsg.data = m_counter;

    // publish the message
    m_pubCounter_->publish(counterMsg);

    // read debug param
    bool isDebugEnabled;
    this->get_parameter<bool>("debug_enabled", isDebugEnabled);

    if (isDebugEnabled && m_counter == 0)
        RCLCPP_INFO(this->get_logger(), "Counter has been reset! Value: %d", m_counter);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<examples::PublisherDemo>());
    rclcpp::shutdown();
    return 0;
}