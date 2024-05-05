#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp" 

using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServerNode : public rclcpp::Node 
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server") 
    {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", 
        std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, _2));
        // We have a create_service method in rclcpp::Node object that allows us to create
        // ROS2 Services. It requires two arguements - name of service and callback.
        RCLCPP_INFO(this->get_logger(), "Service Server has been started.");
    }
 
private:
    void callbackAddTwoInts(
    const example_interfaces::srv::AddTwoInts::Request::SharedPtr request, 
    const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
    }
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
    // In rclcpp, we have a class called service that creates ROS2 services.
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}