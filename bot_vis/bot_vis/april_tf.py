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
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

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
        # TODO i think when defining the actual positions of this transform, itll just be from the yaml
        # Also TODO imort yaml (launch file)
        self.world_2_panda_link0 = TransformStamped()
        self.world_2_panda_link0.header.stamp = self.get_clock().now().to_msg()
        self.world_2_panda_link0.header.frame_id = "world"
        self.world_2_panda_link0.child_frame_id = "panda_link0"
        self.broadcaster = StaticTransformBroadcaster(self)

        # Create a broadcaster to link april tag TF's to robot arm TF's
        self.world_2_cam = TransformStamped()
        self.world_2_cam.header.stamp = self.get_clock().now().to_msg()
        self.world_2_cam.header.frame_id = "world"
        self.world_2_cam.child_frame_id = "camera_link"
        self.broadcaster2 = StaticTransformBroadcaster(self)
        
        # # Create a broadcaster to link === to ===
        # self.world2panda = TransformStamped()
        # self.world2panda.header.stamp = self.get_clock().now().to_msg()
        # self.world2panda.header.frame_id = "world"
        # self.world2panda.child_frame_id = "panda_link0"
        # self.broadcaster2 = TransformBroadcaster(self)

        self.timer = self.create_timer(1/100, self.timer_callback)

    def get_april_2_robot(self):
        """See if april tag is in in end eff. If so use its position to get the TF to base from
        april tag. If not, use saved TF
        """
        try:
            # Get tf to end_eff_tag
            #self.get_logger().error("TEST 1!!!!")
            self.panda_hand_tcp_2_panda_link0 = self.tf_buffer.lookup_transform(
                'panda_link0',
                'panda_hand_tcp',
                rclpy.time.Time())
    # TODO go through this mess and clean it up 
    # TODO just make camera link the world frame so have a parent and child to camera link with no change
    # TODO then get tf of robot base to world (should be from calibration step (double check its right))
    # Overall itll be world as main parent with camera_link and panda_link0 as children and world will just be the same as camera
            self.world_2_panda_link0.transform.translation.x = self.panda_hand_tcp_2_panda_link0.transform.translation.x
            self.world_2_panda_link0.transform.translation.y = self.panda_hand_tcp_2_panda_link0.transform.translation.y
            self.world_2_panda_link0.transform.translation.z = self.panda_hand_tcp_2_panda_link0.transform.translation.z
            self.world_2_panda_link0.transform.rotation.x = self.panda_hand_tcp_2_panda_link0.transform.rotation.x
            self.world_2_panda_link0.transform.rotation.y = self.panda_hand_tcp_2_panda_link0.transform.rotation.y
            self.world_2_panda_link0.transform.rotation.z = self.panda_hand_tcp_2_panda_link0.transform.rotation.z
            self.world_2_panda_link0.transform.rotation.w = self.panda_hand_tcp_2_panda_link0.transform.rotation.w
        except TransformException:
            self.get_logger().info("No tf")

        self.broadcaster.sendTransform(self.world_2_panda_link0)

        self.world_2_cam.transform.translation.x = 0.0
        self.world_2_cam.transform.translation.y = 0.0
        self.world_2_cam.transform.translation.z = 0.0
        self.world_2_cam.transform.rotation.x = 0.0
        self.world_2_cam.transform.rotation.y = 0.0
        self.world_2_cam.transform.rotation.z = 0.0
        self.world_2_cam.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(self.world_2_cam)

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
