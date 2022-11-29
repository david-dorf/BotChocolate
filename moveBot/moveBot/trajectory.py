import rclpy
from rclpy.node import Node
from enum import Enum, auto
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty
import time
from movebot_interfaces.srv import AddBox, GetPlanRqst
from geometry_msgs.msg import Pose
# from moveBot.gripper import Gripper

class State(Enum):
    """Create a state machine to eventually implement planning the entire stored trajectory plan
    sequence, for executing it only once at the end.
    """
    IDLE = auto(),
    PLAN = auto(),
    EXECUTE = auto()

class TrajectoryCaller(Node):
    """Call the plan and execute services from simple_move."""
    def __init__(self):
        super().__init__("trajectory_node")
        self.cbgroup = ReentrantCallbackGroup()
        self.scoop_sub =  self.create_subscription(Pose, 'scoop_pose', self.get_pose_callback, 10)
        self.plan_client = self.create_client(GetPlanRqst,"call_plan",callback_group=self.cbgroup)
        self.cart_client = self.create_client(GetPlanRqst,"call_cart",callback_group=self.cbgroup)
        self.execute_client = self.create_client(Empty,"call_execute",callback_group=self.cbgroup)
        self.request = GetPlanRqst.Request()
        # self.gripper_command=Gripper()
        self.state = State.IDLE


    def get_pose_callback(self, pose_msg):
        """" Callback function of the turtle pose subscriber
            Stores the TurtleSimPose message recieved 
        """
        self.pose=pose_msg 
        print(self.pose)
        

    def send_move_above_request(self):
        """Build the desired IkGoalRqstMsg to be sent over the client to make the robot plan and
        execute a trajectory. This request is the trajectory plan for moving above the object.
        """
        # self.request.start_pos.position and orientation already set as last position by the API
        self.request.goal_pos.position = [self.pose.x,self.pose.y,self.pose.z] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        # self.future contains the plan request
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
     

    def send_move_down_request(self):
        """Generate the trajectory plan for moving down to eventually grip the object."""
        self.request.goal_pos.position = [self.pose.x, self.pose.y, self.pose.z] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_move_up_request(self):
        """Generate the trajectory plan for moving back up after gripping the object."""
        self.request.goal_pos.position = [0.5, 0.5, 0.4] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_move_home_request(self):
        """Generate the trajectory plan for returning to the home position."""
        self.request.goal_pos.position = [0.3, 0.0, 0.5]
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_execute_request(self):
        """Execute the trajectory plan - used in each step of the entire trajectory sequence."""
        self.future = self.execute_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class BoxCaller(Node):
    """Spawn in box objects for the planning scene."""
    def __init__(self):
        super().__init__("box_node")
        self.cbgroup = ReentrantCallbackGroup()
        self.add_box_client = self.create_client(AddBox,"add_box",callback_group=self.cbgroup)
        self.call_box_client = self.create_client(Empty,"call_box",callback_group=self.cbgroup)
        self.clear_box_client = self.create_client(Empty,"clear_all_box",callback_group=self.cbgroup)
        self.request = AddBox.Request()

    def add_box_request(self):
        """Generate the trajectory plan for returning to the home position."""
        # Make a flat box to simulate the table location for collision avoidance
        self.request.x = 0.0
        self.request.y = 0.0
        self.request.z = 0.0
        self.request.l = 5.0
        self.request.w = 5.0
        self.request.h = 0.2
        self.future = self.add_box_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def call_box_request(self):
        """Execute the trajectory plan used in each step of the entire trajectory sequence."""
        self.future = self.call_box_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def clear_box_request(self):
        """Execute the trajectory plan used in each step of the entire trajectory sequence."""
        self.future = self.clear_box_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    # box_client = BoxCaller()
    # box_client.add_box_request()
    # box_client.call_box_request()

    trajectory_client = TrajectoryCaller()
    trajectory_client.send_move_above_request()
    trajectory_client.send_execute_request()
    # trajectory_client.send_move_down_request()
    # trajectory_client.send_execute_request()
    # # Add delay for gripper closing (WIP)
    # time.sleep(1)
    # trajectory_client.send_move_up_request()
    # trajectory_client.send_execute_request()
    # trajectory_client.send_move_home_request()
    # trajectory_client.send_execute_request()
    trajectory_client.destroy_node()

    # box_client.clear_box_request()
    rclpy.shutdown()

if __name__ == '__main__':
    main()