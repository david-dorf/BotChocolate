from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    bot_vis_path = get_package_share_path('bot_vis')
    tag_yaml_path = bot_vis_path / 'config/tag.yaml'
    default_rviz_config_path = bot_vis_path / 'april.rviz'
    calibration_path = bot_vis_path / 'calibration.yaml'

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    DeclareLaunchArgument(
        name="rviz",
        default_value='false',
        choices=['true',
                 'false'],
        description="Flag to launch rviz visualization for seeing April tag tf's"
        )

    DeclareLaunchArgument(
        name="calibration",
        default_value='false',
        choices=['true',
                 'false'],
        description="Flag to calibrate transformations from camera to robot frame"
        )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera')),
            '/launch/rs_launch.py'])
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        remappings=[('image_rect','/camera/color/image_raw'),
                    ('camera_info','/camera/color/camera_info')],
        parameters=[tag_yaml_path]
        )

    april_tf_node = Node(
        package='bot_vis',
        executable='april_tf',
        parameters=[calibration_path]
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=(LaunchConfigurationEquals('rviz', 'true'))
    )

    calibration_node = Node(
        package='bot_vis',
        executable='calibration',
        name='calibration_node',
        condition=(LaunchConfigurationEquals('calibration', 'true'))
    )

    return LaunchDescription([
        realsense_node,
        apriltag_node,
        calibration_node,
        april_tf_node,
        rviz_arg,
        rviz_node
    ])
