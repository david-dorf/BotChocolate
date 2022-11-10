import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

  move_bot_node = Node(
      package='moveBot',
      executable='simple_move',
      name='simple_move',
      )
  ld = LaunchDescription()
  ld.add_action(move_bot_node)
 
  return ld