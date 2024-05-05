#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp" 

class SmartphoneNode : public rclcpp::Node 
{
public:
    SmartphoneNode() : Node("smartphone") 
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::String>("robot_news", 
        10, std::bind(&SmartphoneNode::callbackRobotNews, this, std::placeholders::_1));
        // For any callback in  C++ inside Node, we need to use std::bind first.
        // Also, we need to provide std::placeholders _x, for x = 1 to n, for n
        // number of arguements in callback function.
        RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
    }
 
private:
    void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartphoneNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}