from launch import LaunchDescription
from launch_ros.actions import Node
import sys

def generate_launch_description():
    # Robot IP is passed as command line argument 1
    robot_ip = sys.argv[1]
    # Robot Name is passed as command like argument 2
    robot_name = sys.argv[2]
    return LaunchDescription([
        # Actions
        Node(
            package="cart_pose_server_py",
            executable=robot_name+"_cart_pose",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        ),

        Node(
            package="convey_server_py",
            executable=robot_name+"_convey",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        ),

        Node(
            package="joint_pose_server_py",
            executable=robot_name+"_joints",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        ),

        Node(
            package="schunk_server_py",
            executable=robot_name+"_schunk",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        ),

        Node(
            package="single_joint_server_py",
            executable=robot_name+"_single_joint",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        ),

        # Topics
        Node(
            package="current_cart_py",
            executable=robot_name+"_current_cart",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        ),

        # Node(
        #     package="current_grip_py",
        #     executable=robot_name+"_grip_status",
        #     namespace=robot_name,
        #     parameters=[
        #         { 'robot_ip': robot_ip }
        #     ]
        # ),

        Node(
            package="current_joint_py",
            executable=robot_name+"_current_joint",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        ),

        Node(
            package="move_check_py",
            executable=robot_name+"_movement_status",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        ),

        Node(
            package="prox_check_py",
            executable=robot_name+"_prox_status",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        ),

        Node(
            package="speed_check_py",
            executable=robot_name+"_speed",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        ),

        # Services
        Node(
            package="home_position_py",
            executable=robot_name+"_home",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        ),

        Node(
            package="mount_position_py",
            executable=robot_name+"_mount",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        ),

        Node(
            package="set_speed_py",
            executable=robot_name+"_set_speed",
            namespace=robot_name,
            parameters=[
                { 'robot_ip': robot_ip }
            ]
        )
    ])