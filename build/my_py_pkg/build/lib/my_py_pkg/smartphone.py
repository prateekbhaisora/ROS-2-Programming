#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String  # Even though it is an external package, but we already added its dependency in the package.xml file while creating the publisher 
 
class SmartphoneNode(Node): 
    def __init__(self):
        super().__init__("Smartphone") 
        self.subscriber_ = self.create_subscription(String, 
                            "robot_news", self.callback_robot_news, 10)
        # Uses create_subscription method of Node class to create a subscriber. It takes four
        # arguements - message type, 
        # topic name (should be same as publisher for synchronization purposes) 
        # a callback function to process the message recieved (sent by publisher) by the subscriber during spinning.
        # and queue size.
        # e.g. Here subscriber Smartphone will call the function robot_news whenever this spinning subscriber recieves a message from the publisher.
        self.get_logger().info("Smartphone has been started.")
 
    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)
 
def main(args=None):
    rclpy.init(args=args)
    node = SmartphoneNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()