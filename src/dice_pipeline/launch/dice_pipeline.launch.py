from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='noNAME',
        description='Namespace of the running FANUC robot nodes'
    )
    camera_index_arg = DeclareLaunchArgument(
        'camera_index', default_value='0',
        description='OpenCV camera index for the overhead camera'
    )
    image_save_dir_arg = DeclareLaunchArgument(
        'image_save_dir', default_value='/tmp/dice_images',
        description='Directory where captured die images are saved'
    )

    camera_node = Node(
        package='dice_pipeline',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
            'image_save_dir': LaunchConfiguration('image_save_dir'),
            'roi_x_min': 650,
            'roi_y_min': 350,
            'roi_x_max': 1080,
            'roi_y_max': 700,
        }],
        output='screen',
    )

    master_node = Node(
        package='dice_pipeline',
        executable='master_node',
        name='master_node',
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name'),
        }],
        output='screen',
    )

    return LaunchDescription([
        robot_name_arg,
        camera_index_arg,
        image_save_dir_arg,
        camera_node,
        master_node,
    ])
