from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launchDescription = LaunchDescription()
    colossus_publisher_node = Node(
        package="nav_ros",
        executable="colossus_publisher",
    )
    camera_publisher_node = Node(
        package="camera_ros",
        executable="camera_publisher",
    )
    launchDescription.add_action(colossus_publisher_node)
    launchDescription.add_action(camera_publisher_node)
    return launchDescription