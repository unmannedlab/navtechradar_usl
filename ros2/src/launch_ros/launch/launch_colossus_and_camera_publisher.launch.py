from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([

    Node(
        package="oxb_ros",
        parameters=["../../oxb_ros/config/colossus_and_camera_publisher.yaml"],
        executable="colossus_and_camera_publisher"
    ),

    Node(
        package="tf2_ros",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "camera_image"],
        executable="static_transform_publisher"
    ),

    Node(
        package="rviz2",
        arguments=["-d../../rviz/colossus_and_camera_view.rviz"],
        executable="rviz2"
    )
  ])