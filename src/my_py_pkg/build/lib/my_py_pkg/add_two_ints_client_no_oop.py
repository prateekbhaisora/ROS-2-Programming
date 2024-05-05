#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client_no_oop") 
    client = node.create_client(AddTwoInts, "add_two_ints")
    # create_client() takes two arguements - service type and service name
    while not client.wait_for_service(1.0):
        node.get_logger().warn("Waiting for server Add Two Ints...")
    # This while loop makes the client wait for 1.0 seconds, so that incase server starts a bit late, 
    # the client is still able to successully send the request to the server.
    request = AddTwoInts.Request()
    request.a = 3
    request.b = 8
    future = client.call_async(request)
    # client object has two methods - call() and call_async().
    # It is not preferred to use call() as it remains blocked until a response is recieved which might
    # lead to deadlock in worst case scenario. So, call_async() is more preferred.
    # This call_async() method returns a future object. A future object in Python is one which may posses
    # a value in the future (response, here).
    rclpy.spin_until_future_complete(node, future)
    # The process will keep spinning until future object of node object recieves a response.
    try:
        response = future.result()
        node.get_logger().info(str(request.a) + " + " + str(request.b) + " = " + str(response.sum))
    except Exception as e:
        node.get_logger().error("Service call failed %r " % (e, ))
    # If future result does not recives a response, an exception will be thrown.
    rclpy.shutdown()
    # Also, note that there is no spinning involved here, so the client waits for 1.0 second, 
    # sends its request to the server and dies thereafter. ## This was changed after future object was 
    # introduced.
 
 
if __name__ == "__main__":
    main()