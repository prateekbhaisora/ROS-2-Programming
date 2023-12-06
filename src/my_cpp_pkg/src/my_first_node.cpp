#include"rclcpp/rclcpp.hpp"         // To use ROS Functionalities

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);       // Initalize ROS-2 Communication
    auto node = std::make_shared<rclcpp::Node>("cpp_test"); // Creates a node named cpp_test
    /* For CPP, ROS-2 uses shared pointers everywhere. So, we need not use new and delete
    again and again as everything inside the node will get destroyed once the shared pointer
    goes out of scope.*/
    RCLCPP_INFO(node->get_logger(), "Hello CPP Node");   // To make node print
    rclcpp::spin(node);             // To make node spin; expects a shared pointer as arguement in cpp
    rclcpp::shutdown();             // Terminated ROS-2 Communication
    return 0;
}
