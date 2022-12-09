from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    move_bot_node = Node(
          package='movebot',
          executable='simple_move',
          name='simple_move',
          )
    ld = LaunchDescription()
    ld.add_action(move_bot_node)

    return ld
