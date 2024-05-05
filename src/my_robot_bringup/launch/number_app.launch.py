from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():  # The function name must always be generate_launch_description.
    ld = LaunchDescription()
    
    remap_number_topic = ("number", "my_number")  # Globally created this variable to avoid redundancy
    
    number_publisher_node = Node(   # This will create a node from a Launch file.
        package = "my_py_pkg",
        executable = "number_publisher",
        name = "my_number_publisher",   # Changes name of node at runtime
        remappings = [                  # Changes name of topic at runtime. Must be a list of tuples.
            remap_number_topic
        ],
        parameters = [                  # Changes parameters value at runtime. Must be a list of dictionaries. 
            {"number_to_publish" : 4}, 
            {"publish_frequency" : 5.0}  
        ]
    )
    # Note that these are same arguements that we give to ros2 run, using CLI.
    
    number_counter_node = Node(
        package = "my_cpp_pkg", 
        executable = "number_counter", 
        name = "my_number_counter", 
        remappings = [                       
            remap_number_topic, 
            ("number_count", "my_number_count")
            # Similarly, we can remap srv names at runtime.
        ]
    )
    
    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)
    
    return ld