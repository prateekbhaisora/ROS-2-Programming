#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String   # Imports some inbuilt messages that we downloaded whe we installed ROS2
 
class RobotNewsStationNode(Node): 
    def __init__(self):
        super().__init__("robot_news_station")     # Node name
    # Conventionally, developers use same name for file, node as well as executable.
    
        self.robot_name_ = "CP30"
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        # Uses create_publisher method of Node class to create a publisher_ attribute. It takes three
        # arguements - message type, topic name and a queue size.
        # e.g. Here robot_news_station node will publish on the robot_news topic and a buffer of 10 late messages.

        self.timer_ = self.create_timer(0.5, self.publish_news)    # Invokes publish_news() at 2 Hz
        self.get_logger().info("Robot News Station has been started.")
 
    def publish_news(self):
        msg = String()
        # msg.data = "Hello Robot"        
        # Whenever publish_news function is called a message "Hello Robot" is published on the Topic robot_news
        msg.data = "Hi, this is " + str(self.robot_name_) + " from the robot news station."
        self.publisher_.publish(msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()