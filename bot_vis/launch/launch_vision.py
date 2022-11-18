from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    bot_vis_path = get_package_share_path('bot_vis')
    tag_yaml_path = bot_vis_path / 'tag.yaml'
    
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera')),
            '/launch/rs_launch.py'])
    )

    config = os.path.join(
        get_package_share_path('bot_vis'),
        'tag.yaml'
    )
    # Start turtle robot node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        remappings=[('image_rect','/camera/color/image_raw'),
                    ('camera_info','/camera/color/camera_info')],
        parameters=[config]
        )

    return LaunchDescription([
        realsense_node,
        apriltag_node
    ])
