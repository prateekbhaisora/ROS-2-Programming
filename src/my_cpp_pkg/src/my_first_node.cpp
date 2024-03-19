#include"rclcpp/rclcpp.hpp"         // To use ROS Functionalities

class MyNode: public rclcpp::Node // Creates a public inheritance to rclcpp node
{

public:
    MyNode(): Node("cpp_test"), counter_(0) // Counter variable is initalized to 0
    {
        RCLCPP_INFO(this->get_logger(), "Hello CPP Node");  // To make node print
        timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                         std::bind(&MyNode::timerCallback, this));
    }
    // Wall Timer takes duration and callback function as its arguements.

private:
    void timerCallback()
    {
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
    }

    rclcpp::TimerBase::SharedPtr  timer_;
    // We can use SharePtr, instead of again and again typing std::shared_ptr<rclcpp::TimerBase>
    int counter_;        // counter variable is declared as an attribute
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);       // Initalize ROS-2 Communication
    auto node = std::make_shared<MyNode>(); // Creates a node named cpp_test using constructor MyNode
    /* For CPP, ROS-2 uses shared pointers everywhere. So, we need not use new and delete
    again and again as everything inside the node will get destroyed once the shared pointer
    goes out of scope.*/
    rclcpp::spin(node);             // To make node spin; expects a shared pointer as arguement in cpp
    rclcpp::shutdown();             // Terminated ROS-2 Communication
    return 0;
}
