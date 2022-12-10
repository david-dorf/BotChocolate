import os
import time
import unittest

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions

from std_msgs.msg import Bool

import pytest

import rclpy

from ament_index_python.packages import get_package_share_directory


@pytest.mark.rostest
def generate_test_description():


    cali_node = launch_ros.actions.Node(
        package='bot_vis',
        executable='calibration',
        )

    return (
        launch.LaunchDescription([
            cali_node,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'trutle_robot': cali_node
        }
    )


class TestTalkerListenerLink(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_frequency')

    def tearDown(self):
        self.node.destroy_node()

    def test_frequency(self, launch_service, trutle_robot, proc_output):
        # Expect the talker to publish strings on '/talker_chatter' and also write to stdout
        msgs_rx = []

        sub = self.node.create_subscription(
            Bool,
            '/is_calibrating',
            lambda msg: msgs_rx.append(time.time()),
            10
        )
        sub
        # Wait until the talker transmits two messages over the ROS topic
        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if len(msgs_rx) >= 50:
                break
        t0 = msgs_rx[0]
        tf = msgs_rx[49]
        diff = tf-t0
        self.assertAlmostEqual(diff, 5, delta=0.2)

        # Get 500 sample in less than 10 seconds
        # If frqeuncy is correct, time interval for 500 points from 100 Hz topic is 5 s