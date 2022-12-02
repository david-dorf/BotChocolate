import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from movebot_interfaces.srv import AddBox, GetPlanRqst
from std_srvs.srv import Empty

class BoxCaller(Node):
    """Spawn in box objects for the planning scene."""
    def __init__(self):
        super().__init__("box_node")
        self.cbgroup = ReentrantCallbackGroup()
        self.add_box_client = self.create_client(AddBox,"add_box",callback_group=self.cbgroup)
        self.call_box_client = self.create_client(Empty,"call_box",callback_group=self.cbgroup)
        self.clear_box_client = self.create_client(Empty,"clear_all_box",callback_group=self.cbgroup)
        self.request = AddBox.Request()

        self.timer = self.create_timer(1/100, self.timer_callback)

    def add_box_request(self):
        """Add in the table underneath the base of the robot for collision avoidance."""
        self.request.name = "box1"
        self.request.x = 0.0
        self.request.y = 0.0
        self.request.z = -0.1 - 0.1  # minus 0.1 because of extra collision height
        self.request.l = 0.914
        self.request.w = 0.610
        self.request.h = 0.2
        self.future = self.add_box_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def add_box2_request(self):
        """Add in the table adjacent to the table underneath the base of the robot."""
        self.request.name = "box2"
        self.request.x = 0.37
        self.request.y = 0.69
        self.request.z = -0.19 - 0.1 # minus 0.1 because of extra collision height
        self.request.l = 1.78
        self.request.w = 0.74
        self.request.h = 0.2
        self.future = self.add_box_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def call_box_request(self):
        """Call the boxes into the environment from the add queue."""
        self.future = self.call_box_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def clear_box_request(self):
        """Remove all of the boxes from the simulated environment."""
        self.future = self.clear_box_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def timer_callback(self):
        self.get_logger().info("Spawning boxes")
        self.add_box_request()
        self.call_box_request()
        self.add_box2_request()
        self.call_box_request()



def main(args=None):
    rclpy.init(args=args)
    node = BoxCaller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()









