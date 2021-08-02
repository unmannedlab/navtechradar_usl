from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launchDescription = LaunchDescription()
    colossus_publisher_node = Node(
        package="nav_ros",
        parameters=["./src/nav_ros/config/colossus_publisher.yaml"],
        executable="colossus_publisher",
    )
    camera_publisher_node = Node(
        package="camera_ros",
        parameters=["./src/camera_ros/config/camera_publisher.yaml"],
        executable="camera_publisher",
    )
    launchDescription.add_action(colossus_publisher_node)
    launchDescription.add_action(camera_publisher_node)
    return launchDescription