#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):             # Creates a class inherit from Node object
    
    def __init__(self):                      # Creates a constructor
        super().__init__("py_test")          # Calls super method on node named py_test
        self.counter = 0                     # Added a counter
        self.get_logger().info("Hello ROS2") # Print logs
        self.create_timer(0.5, self.timer_callback) # Uses create_timer method in Node object to call timer_callback function and print Hello every 2 Hz = 0.5 seconds
        
    def timer_callback(self):
        self.counter += 1
        self.get_logger().info("Hello " + str(self.counter))   # Timer function to print Hello and count

def main(args=None):
    rclpy.init(args = args)     # Starts ROS2 Communication
    node = MyNode()             # Creates a node named py_test using MyNode class Constructor 
    rclpy.spin(node)            # Keeps our node alive, stops spinning with Ctrl+C
    rclpy.shutdown()            # Terminates ROS2 Communication

if __name__ == "__main__":
    main()
