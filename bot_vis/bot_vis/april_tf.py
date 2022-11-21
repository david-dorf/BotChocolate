# Basic ROS 2 program to subscribe to real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image, CameraInfo  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped


class AprilTF(Node):
    """
    Node that publishes the current positions of the kettle, cup, scooper, and stirrer relative to
    the robot frame. 
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('april_tf')

        # Create publishers to publish the x,y,z,R,P,Y position of each aspect of the botcholate
        # setup
        self.scoop_pub = self.create_publisher(Pose, 'scoop_pos', 10)
        self.cup_pub = self.create_publisher(Pose, 'cup_pos', 10)
        self.kettle_pub = self.create_publisher(Pose, 'kettle_pos', 10)
        self.stirrer_pub = self.create_publisher(Pose, 'stirrer_pos', 10)

        # Create a listener to recieve the TF's from each tag to the camera
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a broadcaster to link april tag TF's to robot arm TF's
        self.ee_tag_2_panda_hand_tcp = TransformStamped()
        self.ee_tag_2_panda_hand_tcp.header.stamp = self.get_clock().now().to_msg()
        self.ee_tag_2_panda_hand_tcp.header.frame_id = "panda_hand_tcp"
        self.ee_tag_2_panda_hand_tcp.child_frame_id = "end_eff_tag"
        self.broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1/100, self.timer_callback)

    def calibrate(self):
        """Obtain TF from camera to robot frame
        """
        
    def get_april_2_robot(self):
        """See if april tag is in in end eff. If so use its position to get the TF to base from
        april tag. If not, use saved TF
        """
        # try:
        #     # Get tf to end_eff_tag
        #     self.ee_tag_2_panda_hand_tcp = self.tf_buffer.lookup_transform(
        #         'panda_hand_tcp',
        #         'panda_link0',
        #         rclpy.time.Time())

        #     # Average TF over a couple frames and save to file
        #     self.get_logger().error(self.ee_tag_2_panda_hand_tcp)
        # except:
        #     # Get TF from file
        #     self.get_logger().error("Not seeing calibrate")
        #     pass
        self.ee_tag_2_panda_hand_tcp.transform.translation.x = 0.0
        self.ee_tag_2_panda_hand_tcp.transform.translation.y = 0.0
        self.ee_tag_2_panda_hand_tcp.transform.translation.z = 0.0

        self.broadcaster.sendTransform(self.ee_tag_2_panda_hand_tcp)

    def timer_callback(self):
        """
        Callback function.
        """
        self.get_april_2_robot()
        # Need to broadcast tf from ee to panda_link0
        try:
            scoop_2_base = self.tf_buffer.lookup_transform(
                'scoop',
                'panda_link0',
                rclpy.time.Time())
        except:
            pass

        try:
            cup_2_base = self.tf_buffer.lookup_transform(
                'cup',
                'panda_link0',
                rclpy.time.Time())
        except:
            pass

        try:
            kettle_2_base = self.tf_buffer.lookup_transform(
                'kettle',
                'panda_link0',
                rclpy.time.Time())
        except:
            pass

        try:
            stirrer_2_base = self.tf_buffer.lookup_transform(
                'stirrer',
                'panda_link0',
                rclpy.time.Time())
        except:
            pass


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    april_tf = AprilTF()

    # Spin the node so the callback function is called.
    rclpy.spin(april_tf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    april_tf.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
