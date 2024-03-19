#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp" 
 
class NumberCounterNode : public rclcpp::Node 
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0) 
    {
        // Declaring and initalizing a custom parameter, with a default value.
        // this->declare_parameter("param_3_", 7);
        // parameter = this->get_parameter("param_3_ ").as_int(); 
        // It is always preferred to declare/create publishers before subscribers.
        counter_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 
        10);
        number_subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 
        10, std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Number Counter has been started.");
    }
 
private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        auto newMsg = example_interfaces::msg::Int64();
        newMsg.data = counter_;
        counter_publisher_->publish(newMsg);
    }
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr counter_publisher_;
    int counter_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}