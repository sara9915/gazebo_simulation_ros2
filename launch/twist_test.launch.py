from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    slider0_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["-0.0511367", "-0.381597", "0.156755", "0", "0", "0", "base_link", "slider0"],
                       output = "screen")

    slider_tf = Node(package = "tf2_ros", 
                    executable = "static_transform_publisher",
                    arguments = ["0", "0", "0", "-0.52", "0", "0", "slider0", "slider"],
                    output = "screen")
    
    twist_node = Node(package = "uclv_yaskawa_simulation", 
                       executable = "twist_ref_pub",
                       output = "screen")

    ld.add_action(twist_node)
    ld.add_action(slider0_tf)
    ld.add_action(slider_tf)


    return ld