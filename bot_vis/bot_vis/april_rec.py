import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import apriltag #apriltag library

class AprilRec(Node):
    def __init__(self):
        super().__init__('april_rec')
        self.subscription = self.create_subscription(
        Image, 
        '/camera/color/image_raw', 
        self.listener_callback, 
        10)

        self.br = CvBridge()

    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)
        gray = cv2.cvtColor(current_frame, cv2.COLOR_RGB2GRAY)
        color = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        options = apriltag.DetectorOptions(families="tag36h11",refine_pose=True)
        detector = apriltag.Detector(options)
        results = detector.detect(gray)

        cv2.imshow("Gray", gray)
        #cv2.waitKey(1)

        for r in results:
# extract the bounding box (x, y)-coordinates for the AprilTag
# and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            print(f"{r.refine_pose}")
# draw the bounding box of the AprilTag detection
            cv2.line(color, ptA, ptB, (0, 255, 0), 2)
            cv2.line(color, ptB, ptC, (0, 255, 0), 2)
            cv2.line(color, ptC, ptD, (0, 255, 0), 2)
            cv2.line(color, ptD, ptA, (0, 255, 0), 2)
# draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(color, (cX, cY), 5, (0, 0, 255), -1)
# draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            R = str(r.tag_id)
            cv2.putText(color, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(color, R, (ptA[0], ptA[1] - 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("[INFO] tag family: {}".format(tagFamily))
# show the output image after AprilTag detection
        cv2.imshow("Image", color)
        cv2.waitKey(1)
    

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = AprilRec()
  
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