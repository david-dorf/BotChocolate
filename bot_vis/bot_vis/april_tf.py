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
from std_msgs.msg import Bool
from rcl_interfaces.msg import ParameterDescriptor

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
        self.scoop_pub = self.create_publisher(Pose, 'scoop_xzy', 10)
        self.cup_pub = self.create_publisher(Pose, 'cup_xyz', 10)
        self.kettle_pub = self.create_publisher(Pose, 'kettle_xyz', 10)
        self.stirrer_pub = self.create_publisher(Pose, 'stirrer_xyz', 10)



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
        self.calibrate_flag_sub = self.create_subscription(
            Bool, '/is_calibrating', self.calibrate_flag_cb, 10)
        self.calibrate_flag = False

        # Get transformation params from calibration
        self.declare_parameter("x", 0.0, ParameterDescriptor(
            description="The x translation from base to world frame"))
        self.declare_parameter("y", 0.0, ParameterDescriptor(
            description="The y translation from base to world frame"))
        self.declare_parameter("z", 0.0, ParameterDescriptor(
            description="The z translation from base to world frame"))
        self.declare_parameter("x_q", 0.0, ParameterDescriptor(
            description="The x rotation from base to world frame"))
        self.declare_parameter("y_q", 0.0, ParameterDescriptor(
            description="The y rotation from base to world frame"))
        self.declare_parameter("z_q", 0.0, ParameterDescriptor(
            description="The z rotation from base to world frame"))
        self.declare_parameter("w_q", 0.0, ParameterDescriptor(
            description="The w rotation from base to world frame"))
        self.cali_trans_x = self.get_parameter(
            "x").get_parameter_value().double_value
        self.cali_trans_y = self.get_parameter(
            "y").get_parameter_value().double_value
        self.cali_trans_z = self.get_parameter(
            "z").get_parameter_value().double_value
        self.cali_rot_x = self.get_parameter(
            "x_q").get_parameter_value().double_value
        self.cali_rot_y = self.get_parameter(
            "y_q").get_parameter_value().double_value
        self.cali_rot_z = self.get_parameter(
            "z_q").get_parameter_value().double_value
        self.cali_rot_w = self.get_parameter(
            "w_q").get_parameter_value().double_value

        #static broadcaster
        #static frames for gripper use
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.kettle_adapter_tf = TransformStamped()
        self.kettle_adapter_tf.header.stamp = self.get_clock().now().to_msg()
        self.kettle_adapter_tf.header.frame_id = "kettle"
        self.kettle_adapter_tf.child_frame_id = "kettle_adaptor"
        self.kettle_adapter_tf.transform.translation.x = 0.1651
        self.kettle_adapter_tf.transform.translation.y = 0.1016
        self.kettle_adapter_tf.transform.translation.z = -0.0762

        self.cup_center_tf = TransformStamped()
        self.cup_center_tf.header.stamp = self.get_clock().now().to_msg()
        self.cup_center_tf.header.frame_id = "cup"
        self.cup_center_tf.child_frame_id = "cup_center"
        #cup_center_tf.transform.translation.x = 0.1651
        #cup_center_tf.transform.translation.y = 0.1016
        self.cup_center_tf.transform.translation.z = -0.0381

        self.timer = self.create_timer(1/100, self.timer_callback)

    def calibrate_flag_cb(self, data):
        self.calibrate_flag = data

    def get_april_2_robot(self):
        """See if april tag is in in end eff. If so use its position to get the TF to base from
        april tag. If not, use saved TF
        """

        self.world_2_panda_link0.transform.translation.x = self.cali_trans_x
        self.world_2_panda_link0.transform.translation.y = self.cali_trans_y
        self.world_2_panda_link0.transform.translation.z = self.cali_trans_z
        self.world_2_panda_link0.transform.rotation.x = self.cali_rot_x
        self.world_2_panda_link0.transform.rotation.y = self.cali_rot_y
        self.world_2_panda_link0.transform.rotation.z = self.cali_rot_z
        self.world_2_panda_link0.transform.rotation.w = self.cali_rot_w


        self.broadcaster.sendTransform(self.world_2_panda_link0)

        # World and camera link are at same location
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
        if not self.calibrate_flag: # IF not calibrating
            self.get_april_2_robot()
            self.static_broadcaster.sendTransform(self.kettle_adapter_tf)
            self.static_broadcaster.sendTransform(self.cup_center_tf)

        # Need to broadcast tf from ee to panda_link0
        try:
            scoop_2_base = self.tf_buffer.lookup_transform(
                'scoop',
                'panda_link0',
                rclpy.time.Time())
            scoop_xzy = Pose()
            scoop_xzy.position.x = scoop_2_base.transform.translation.x
            scoop_xzy.position.y = scoop_2_base.transform.translation.y
            scoop_xzy.position.z = scoop_2_base.transform.translation.z
            self.scoop_pub.publish(scoop_xzy)
        except:
            pass

        try:
            cup_2_base = self.tf_buffer.lookup_transform(
                'cup_center',
                'panda_link0',
                rclpy.time.Time())
            cup_xzy = Pose()
            cup_xzy.position.x = cup_2_base.transform.translation.x
            cup_xzy.position.y = cup_2_base.transform.translation.y
            cup_xzy.position.z = cup_2_base.transform.translation.z
            self.cup_pub.publish(cup_xzy)
        except:
            pass

        try:
            kettle_2_base = self.tf_buffer.lookup_transform(
                'kettle',
                'panda_link0',
                rclpy.time.Time())
            kettle_xzy = Pose()
            kettle_xzy.position.x = kettle_2_base.transform.translation.x
            kettle_xzy.position.y = kettle_2_base.transform.translation.y
            kettle_xzy.position.z = kettle_2_base.transform.translation.z
            self.kettle_pub.publish(kettle_xzy)
        except:
            pass

        try:
            # TODO
            stirrer_2_base = self.tf_buffer.lookup_transform(
                'stirrer',
                'panda_link0',
                rclpy.time.Time())
            stirrer_xzy = Pose()
            stirrer_xzy.position.x = stirrer_2_base.transform.translation.x
            stirrer_xzy.position.y = stirrer_2_base.transform.translation.y
            stirrer_xzy.position.z = stirrer_2_base.transform.translation.z
            self.kettle_pub.publish(stirrer_xzy)
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
