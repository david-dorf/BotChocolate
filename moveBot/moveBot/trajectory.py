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
    GO_SCOOP=auto(),
    PLAN = auto(),
    EXECUTE = auto()

class TrajectoryCaller(Node):
    """Call the plan and execute services from simple_move."""
    def __init__(self):
        super().__init__("trajectory_node")
        self.cbgroup = ReentrantCallbackGroup()
        self.scoop_sub =  self.create_subscription(Pose, 'scoop_xzy', self.get_pose_callback, 10)
        self.plan_client = self.create_client(GetPlanRqst,"call_plan",callback_group=self.cbgroup)
        self.cart_client = self.create_client(GetPlanRqst,"call_cart",callback_group=self.cbgroup)
        self.execute_client = self.create_client(Empty,"call_execute",callback_group=self.cbgroup)
        self.request = GetPlanRqst.Request()
        # self.pose=Pose()

        self.timer     = self.create_timer(1/100, self.timer_callback)
        # self.gripper_command=Gripper()
        self.state = State.IDLE

        self.home_x = 0.3
        self.home_y = 0.0
        self.home_z = 0.5
        self.home_roll = 3.14
        self.home_pitch = 0.0
        self.home_yaw = 0.0


    def get_pose_callback(self, pose_msg):
        """" Callback function of the turtle pose subscriber
            Stores the TurtleSimPose message recieved
        """

        # print(pose_msg)
        self.pose=pose_msg
        # print(self.pose.position.x)/


    # def send_move_above_request(self):
    #     """Build the desired IkGoalRqstMsg to be sent over the client to make the robot plan and
    #     execute a trajectory. This request is the trajectory plan for moving above the object.
    #     """
    #     # self.request.start_pos.position and orientation already set as last position by the API
    #     self.request.goal_pos.position =  [self.pose.position.x,self.pose.position.y,self.pose.position.z] # placeholder values, replace with CV
    #     self.request.goal_pos.orientation = []
    #     self.request.is_xyzrpy = True
    #     self.request.execute_now = False
    #     # self.future contains the plan request
    #     self.future = self.cart_client.call_async(self.request)
    #     rclpy.spin_until_future_complete(self, self.future)
    #     return self.future.result()


    def send_move_above_request(self):
        """Build the desired IkGoalRqstMsg to be sent over the client to make the robot plan and
        execute a trajectory. This request is the trajectory plan for moving above the object.
        """
        self.request.goal_pos.position = [0.3, 0.5, 0.4] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_rotate_request(self):
        """Rotate end effector to desired orientation."""
        self.request.goal_pos.position = []
        self.request.goal_pos.orientation = [self.home_roll, self.home_pitch + 1.0, self.home_yaw]
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.plan_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_rotate_request2(self):
        """Rotate end effector back to original orientation."""
        self.request.goal_pos.position = []
        self.request.goal_pos.orientation = [self.home_roll, self.home_pitch, self.home_yaw]
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.plan_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_move_down_request(self):
        """Generate the trajectory plan for moving down to eventually grip the object."""
        self.request.goal_pos.position = [0.3, 0.5, -0.05] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_move_up_request(self):
        """Generate the trajectory plan for moving back up after gripping the object."""
        self.request.goal_pos.position = [0.3, 0.5, 0.4] # placeholder values, replace with CV
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

    def send_stir1_request(self):
        """Plan path to the first point in the diamond path for stirring."""
        self.request.goal_pos.position = [0.33, 0.5, -0.05] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_stir2_request(self):
        """Plan path to the second point in the diamond path for stirring."""
        self.request.goal_pos.position = [0.3, 0.53, -0.05] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_stir3_request(self):
        """Plan path to the third point in the diamond path for stirring."""
        self.request.goal_pos.position = [0.27, 0.5, -0.05] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_stir4_request(self):
        """Plan path to the fourth point in the diamond path for stirring."""
        self.request.goal_pos.position = [0.3, 0.47, -0.05] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_stir5_request(self):
        """Plan path to the center point in the diamond path for stirring."""
        self.request.goal_pos.position = [0.3, 0.5, -0.05] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    def timer_callback(self):
        box_client = BoxCaller()
        box_client.add_box_request()
        box_client.call_box_request()
        box_client.add_box2_request()
        box_client.call_box_request()
        try:
            # Move the end effector above the object
            self.send_move_above_request()
            self.send_execute_request()

            # Move the end effector down to the object
            self.send_move_down_request()
            self.send_execute_request()

            self.send_move_above_request()
            self.send_execute_request()
            self.send_rotate_request()
            self.send_execute_request()
            time.sleep(2)
            self.send_rotate_request2()
            self.send_execute_request()

            # Move the end effector down to the object
            self.send_move_down_request()
            self.send_execute_request()

            # Add delay for gripper closing (insert gripper service call here)
            time.sleep(1)

            # Move the end effector back up
            self.send_move_up_request()
            self.send_execute_request()

            # Tilt the object to pour out its contents
            self.send_rotate_request()
            self.send_execute_request()
            time.sleep(2)
            self.send_rotate_request2()
            self.send_execute_request()

            # Loop a diamond path stirring action
            for i in range(4):
                self.send_stir1_request()
                self.send_execute_request()
                self.send_stir2_request()
                self.send_execute_request()
                self.send_stir3_request()
                self.send_execute_request()
                self.send_stir4_request()
                self.send_execute_request()
            self.send_stir5_request()
            self.send_execute_request()

            # Move the end effector back up
            self.send_move_up_request()
            self.send_execute_request()
            # Return the end effector to its home configuration
            self.send_move_home_request()
            self.send_execute_request()
            box_client.clear_box_request()

        except:
            pass



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



def main(args=None):
    rclpy.init(args=args)

    self = TrajectoryCaller()
    rclpy.spin(self)
    rclpy.shutdown()

if __name__ == '__main__':
    main()