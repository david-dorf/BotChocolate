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
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point


class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.april_pub = self.create_publisher(Image, 'image_rect', 10)
        self.cam_info = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.cam_info_CB, 10)
        self.april_info_pub = self.create_publisher(
            CameraInfo, 'camera_info', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.coord_10_17 = Point()

    def cam_info_CB(self, data):
        self.april_info_pub.publish(data)

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        im_rgb = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)
        im_cann = cv2.Canny(im_rgb, 100, 100)
        self.publisher_.publish(self.br.cv2_to_imgmsg(im_cann))
        self.april_pub.publish(data)
        # Display image
        cv2.imshow("camera", im_rgb)
        try:
            t = self.tf_buffer.lookup_transform(
                'tag36h11:10', 'tag36h11:17', rclpy.time.Time())
            self.coord_10_17.x = t.transform.translation.x
            self.coord_10_17.y = t.transform.translation.y
            self.coord_10_17.z = t.transform.translation.z
            self.get_logger().info(f"x {self.coord_10_17.x} y {self.coord_10_17.y} z {self.coord_10_17.z}")

        except TransformException:
                self.get_logger().info("No tf")
        cv2.waitKey(1)

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
