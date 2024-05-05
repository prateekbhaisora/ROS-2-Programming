#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_oop"); 
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    // Note that the arguement must be same as the server name, else client-server won't be able to 
    // communicate with each other.
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(node->get_logger(), "Waiting for the server to be up...");
    }
    // The above while loop will make the client wait 1 second for the server to get up
    // and send a response.
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    // This will create a shared pointer for requests.
    request->a = 3;
    request->b = 8;
    auto future = client->async_send_request(request);
    // We will recieve a shared future of a shared pointer of the response.
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "%ld + %ld = %ld", request->a, request->b, future.get()->sum);
    }
    // If response is successfully recieved, then print the computation performed.
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Error while calling the service!");
    }
    rclcpp::shutdown();
    // Note that we will not spin the client. We will just create the client, 
    // send a request and then will simply wait until the server sends a response.
    return 0;
}