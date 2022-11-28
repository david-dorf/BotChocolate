# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from ament_index_python.packages import get_package_share_path, get_package_prefix
import yaml
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from std_msgs.msg import Bool

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

        # Path for the calibration file
        self.bot_vis_path = get_package_share_path('bot_vis')
        self.calibration_path = str(self.bot_vis_path) + '/calibration.yaml'

        # Lists to save the x,y,z and quaternions
        self.x_list = []
        self.y_list = []
        self.z_list = []
        self.quat_x_list = []
        self.quat_y_list = []
        self.quat_z_list = []
        self.quat_w_list = []
        self.time_count = 0
        self.done = False

        # Define broadcaster from the calibration tag to the base of the robot
        self.end_eff_tag_2_panda_link0 = TransformStamped()
        self.end_eff_tag_2_panda_link0.header.stamp = self.get_clock().now().to_msg()
        self.end_eff_tag_2_panda_link0.header.frame_id = "end_eff_tag"
        self.end_eff_tag_2_panda_link0.child_frame_id = "panda_link0"
        self.broadcaster = StaticTransformBroadcaster(self)

        # Create a publisher that indicates if the calibration node is running
        self.flag_pub = self.create_publisher(Bool, '/is_calibrating', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)


    def generate_yaml(self, x_in, y_in, z_in, x_quat_in, y_quat_in, z_quat_in, w_quat_in):
        """
        Take in a x, y, z position with a quaternion angle and returns a dictionary that will
        be used to generate the calibration yaml.

        Args:
        ----
            x_in (float): x position of base relative to world frame
            y_in (float): y position of base relative to world frame
            z_in (float): z position of base relative to world frame
            x_quat_in (float): x rotation of base relative to world frame
            y_quat_in (float): y rotation of base relative to world frame
            z_quat_in (float): z rotation of base relative to world frame
            w_quat_in (float): w rotation of base relative to world frame

        Returns:
        -------
            dict: Dictionary with values for yaml file
        """
        return dict(x = x_in,
                    y = y_in,
                    z = z_in,
                    x_q = x_quat_in,
                    y_q = y_quat_in,
                    z_q = z_quat_in,
                    w_q = w_quat_in)

    def append_list(self):
        """Add the the current TF locations to a list of position values to be averaged later.
        """
        self.x_list.append(self.cam_2_panda_link0.transform.translation.x)
        self.y_list.append(self.cam_2_panda_link0.transform.translation.y)
        self.z_list.append(self.cam_2_panda_link0.transform.translation.z)
        self.quat_x_list.append(self.cam_2_panda_link0.transform.rotation.x)
        self.quat_y_list.append(self.cam_2_panda_link0.transform.rotation.y)
        self.quat_z_list.append(self.cam_2_panda_link0.transform.rotation.z)
        self.quat_w_list.append(self.cam_2_panda_link0.transform.rotation.w)

    def link_frames(self):
        """
        Set the TF from the end effector tag to the panda equal to the TF from the panda hand tcp
        frame since the end effector tag is in the same place as the panda hand tcp frame.
        """
        self.end_eff_tag_2_panda_link0.transform.translation.x = \
            self.panda_hand_tcp_2_panda_link0.transform.translation.x

        self.end_eff_tag_2_panda_link0.transform.translation.y = \
            self.panda_hand_tcp_2_panda_link0.transform.translation.y

        self.end_eff_tag_2_panda_link0.transform.translation.z = \
            self.panda_hand_tcp_2_panda_link0.transform.translation.z

        self.end_eff_tag_2_panda_link0.transform.rotation.x = \
            self.panda_hand_tcp_2_panda_link0.transform.rotation.x

        self.end_eff_tag_2_panda_link0.transform.rotation.y = \
            self.panda_hand_tcp_2_panda_link0.transform.rotation.y

        self.end_eff_tag_2_panda_link0.transform.rotation.z = \
            self.panda_hand_tcp_2_panda_link0.transform.rotation.z

        self.end_eff_tag_2_panda_link0.transform.rotation.w = \
            self.panda_hand_tcp_2_panda_link0.transform.rotation.w

        self.broadcaster.sendTransform(self.end_eff_tag_2_panda_link0)

    def get_tf(self):
        """
        Obtain the transform from the camera to the end_effector april tag, append it to a list,
        and average the tfs
        """
        # Listen to TF from camera to end-effector april tag
        try:
            # Get tf from camera to end_eff_tag
            self.cam_2_ee = self.tf_buffer.lookup_transform(
                'camera_link',
                'end_eff_tag',
                rclpy.time.Time())

            # Get TF from panda_hand_tcp to panda_link0
            self.panda_hand_tcp_2_panda_link0 = self.tf_buffer.lookup_transform(
                'panda_hand_tcp',
                'panda_link0',
                rclpy.time.Time())

            # Link frames
            self.link_frames()
            self.cam_2_panda_link0 = self.tf_buffer.lookup_transform(
                'camera_link',
                'panda_link0',
                rclpy.time.Time())

            self.append_list()

            self.time_count += 1
            self.get_logger().error("Calibrating")
        except:
            # Get TF from file
            self.get_logger().error("Tag not detected! Make sure tag in camera view.")
            pass

    def average_points(self, x_list, y_list, z_list, quat_x_list, 
                       quat_y_list, quat_z_list, quat_w_list):
        """Average several lists of points.

        Args:
        ----
            x_list (list): list of x points to be averaged
            y_list (list): list of y points to be averaged
            z_list (list): list of z points to be averaged
            quat_x_list (list): list of rotation x points to be averaged
            quat_y_list (list): list of rotation y points to be averaged
            quat_z_list (list): list of rotation z points to be averaged
            quat_w_list (list): list of rotation w points to be averaged

        Returns:
        -------
            float: the averaged x,y,z and rotation values
        """
        try:
            x = sum(x_list) / len(x_list)
            y = sum(y_list) / len(y_list)
            z = sum(z_list) / len(z_list)
            x_quat = sum(quat_x_list) / len(quat_x_list)
            y_quat = sum(quat_y_list) / len(quat_y_list)
            z_quat = sum(quat_z_list) / len(quat_z_list)
            w_quat = sum(quat_w_list) / len(quat_w_list)
        except:
            pass
        return x, y, z, x_quat, y_quat, z_quat, w_quat

    def timer_callback(self):
        """
        Callback function.
        """
        # Let other nodes know that we are calibrating
        flag = Bool()
        flag.data = True
        self.flag_pub.publish(flag)

        # Get transforms over a 5 second interval
        if self.time_count < 50:
            self.get_tf()
        else: # Average TF readings and put the average in a yaml
            x, y, z, x_quat, y_quat, z_quat, w_quat = \
                self.average_points(self.x_list, self.y_list, self.z_list, self.quat_x_list, 
                                    self.quat_y_list, self.quat_z_list, self.quat_w_list)

            data = dict( april_tf = dict(
            ros__parameters = 
                self.generate_yaml(x, y, z, x_quat, y_quat, z_quat, w_quat)
            ))

            with open(str(self.calibration_path), 'w') as outfile:
                outfile.write(yaml.safe_dump(data, default_flow_style=False))
            self.get_logger().info("Done Calibrating")
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    calibration = Calibration()
    rclpy.spin(calibration)
    calibration.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
