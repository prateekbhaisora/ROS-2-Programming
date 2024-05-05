#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial # This will allow us to add more than one arguements to our add_done_callback()
from example_interfaces.srv import AddTwoInts
 
class AddTwoIntsClientNode(Node): 
    def __init__(self):
        super().__init__("add_two_ints_client") 
        self.call_add_two_ints_server(6, 7)
        self.call_add_two_ints_server(2, 5)
        self.call_add_two_ints_server(9, 4)
    
    def call_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server Add Two Ints...")
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_add_two_ints, a = a, b = b))
        # So, instead of spinning until future is complete, we rather call a callback(), that
        # deals with the future.
        
    def callback_call_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(str(a) + " + " + str(b) + " = " + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r " % (e, ))
        
def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode() 
    rclpy.spin(node)
    # Note that here we are already spinning the node. So, it is not recommeded to write a code
    # (e.g. a while loop) inside an already spinning node to spin until future is complete. 
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()