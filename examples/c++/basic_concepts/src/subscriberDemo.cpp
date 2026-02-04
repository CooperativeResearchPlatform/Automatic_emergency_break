#include "basic_concepts/subscriberDemo.hpp"


examples::SubscriberDemo::SubscriberDemo() : Node("subscriber_demo")
{
    // declare and retrieve parameters
    this->declare_parameter<std::string>("topic_name", "counter");
    std::string topicName;
    this->get_parameter<std::string>("topic_name", topicName);

    // create subscriber and attach callbacks to the counterCallback method
    m_subCounter_ = this->create_subscription<std_msgs::msg::UInt8>(
        topicName,
        3,
        std::bind(&SubscriberDemo::counterCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "subscriber_demo node has been started");
}


void examples::SubscriberDemo::counterCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    // display topic messages on the terminal
    RCLCPP_INFO(this->get_logger(), "Received counter: %d", msg->data);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<examples::SubscriberDemo>());
    rclcpp::shutdown();
    return 0;
}
