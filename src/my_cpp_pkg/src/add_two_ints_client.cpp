#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp" 

class AddTwoIntsClientNode : public rclcpp::Node 
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client") 
    {
        thread1_ = std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4));
        // If we directly call the callback function e.g. callAddTwoIntsService(1, 4) in this thread, 
        // then the future.get() will block this thread until a response is recived. 
        // But, two recieve a response, the code needs to get out of the constructor 
        // and execute the spin. So, we need to call the callAddTwoIntsService(1, 4) in a different 
        // thread.
        /*
        To send multiple calls to the server, we need to implement a thread pool in C++.
        One simple solution is to create an vector of threads as follows:
        */
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 3, 4)));
        threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 6, 5)));
    }

    void callAddTwoIntsService(int a, int b)
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            // The response recived will always be a shared pointer.
            RCLCPP_INFO(this->get_logger(), "%d + %d = %ld", a, b, response->sum);
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed!");    
        }
        // future.get() will block the thread until a response is recieved. Also, it might raise an 
        // exception in case of an error, so we need to put it inside a try-catch structure.
    }   
 
private:
    std::thread thread1_;
    std::vector<std::thread> threads_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}