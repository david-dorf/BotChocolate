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
from ament_index_python.packages import get_package_share_path, get_package_prefix
import yaml


class Calibration(Node):
    """
    Node that finds transform from camera to end_effector tag and saves the pose of the calibration
    tag in a yaml file. 
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('calibration')

        # Create a listener to recieve the TF's from each tag to the camera
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a broadcaster to link april tag TF's to robot arm TF's
        self.cam_2_ee = TransformStamped()
        self.cam_2_ee.header.stamp = self.get_clock().now().to_msg()
        self.cam_2_ee.header.frame_id = "panda_hand_tcp"
        self.cam_2_ee.child_frame_id = "end_eff_tag"
        self.broadcaster = TransformBroadcaster(self)
        self.bot_vis_path = get_package_share_path('bot_vis')
        self.calibration_path = str(self.bot_vis_path) + '/calibration.yaml'
        self.x_list = []
        self.y_list = []
        self.z_list = []
        self.quat_x_list = []
        self.quat_y_list = []
        self.quat_z_list = []
        self.quat_w_list = []
        self.time_count = 0
        self.done = False
        self.timer = self.create_timer(0.1, self.timer_callback)


    def generate_yaml(self, x_in, y_in, z_in, x_quat_in, y_quat_in, z_quat_in, w_quat_in):
        return dict(x = x_in,
                    y = y_in,
                    z = z_in,
                    x_q = x_quat_in,
                    y_q = y_quat_in,
                    z_q = z_quat_in,
                    w_q = w_quat_in)

    def append_list(self):
        self.x_list.append(self.cam_2_ee.transform.translation.x)
        self.y_list.append(self.cam_2_ee.transform.translation.y)
        self.z_list.append(self.cam_2_ee.transform.translation.z)
        self.quat_x_list.append(self.cam_2_ee.transform.rotation.x)
        self.quat_y_list.append(self.cam_2_ee.transform.rotation.y)
        self.quat_z_list.append(self.cam_2_ee.transform.rotation.z)
        self.quat_w_list.append(self.cam_2_ee.transform.rotation.w)

    def get_tf(self):
        """Obtain the transform from the camera to the end_effector april tag, append it to a list,
        and average the tfs
        """
        # Listen to TF from camera to end-effector april tag
        try:
            # Get tf to end_eff_tag
            self.cam_2_ee = self.tf_buffer.lookup_transform(
                'camera_link',
                'end_eff_tag',
                rclpy.time.Time())
            self.append_list()
            self.time_count += 1
            self.get_logger().error("Calibrating")
        except:
            # Get TF from file
            self.get_logger().error("Tag not detected! Make sure tag in camera view.")
            pass

    def average_points(self):
        try:
            x = sum(self.x_list) / len(self.x_list)
            y = sum(self.y_list) / len(self.y_list)
            z = sum(self.z_list) / len(self.z_list)
            x_quat = sum(self.quat_x_list) / len(self.quat_x_list)
            y_quat = sum(self.quat_y_list) / len(self.quat_y_list)
            z_quat = sum(self.quat_z_list) / len(self.quat_z_list)
            w_quat = sum(self.quat_w_list) / len(self.quat_w_list)
        except:
            pass
        return x, y, z, x_quat, y_quat, z_quat, w_quat

    def timer_callback(self):
        """
        Callback function.
        """

        # Get transforms over a 5 second interval
        if self.time_count < 50:
            self.get_tf()
        else: # Average TF readings and put the average in a yaml
            x, y, z, x_quat, y_quat, z_quat, w_quat = self.average_points()
            data = dict(
            calibration = tuple([
                self.generate_yaml(x, y, z, x_quat, y_quat, z_quat, w_quat)
            ]))

            with open(str(self.calibration_path), 'w') as outfile:
                outfile.write(yaml.safe_dump(data, default_flow_style=False))
            self.get_logger().info("Done Calibrating")
            self.destroy_node()





def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    calibration = Calibration()

    # Spin the node so the callback function is called.
    rclpy.spin(calibration)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    calibration.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
