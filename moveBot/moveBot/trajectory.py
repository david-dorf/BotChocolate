import rclpy
from rclpy.node import Node
from enum import Enum, auto
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty
from movebot_interfaces.srv import IkGoalRqst, AddBox, GetPlanRqst
from movebot_interfaces.msg import IkGoalRqstMsg

class State(Enum):
    IDLE = auto(),
    PLAN = auto(),
    EXECUTE = auto()

class TrajectoryCaller(Node):
    """Call the plan and execute services from simple_move."""
    def __init__(self):
        super().__init__("trajectory_node")
        self.cbgroup = ReentrantCallbackGroup()
        self.plan_client = self.create_client(GetPlanRqst,"call_plan",callback_group=self.cbgroup)
        self.cart_client = self.create_client(GetPlanRqst,"call_cart",callback_group=self.cbgroup)
        self.execute_client = self.create_client(Empty,"call_execute",callback_group=self.cbgroup)
        self.request = GetPlanRqst.Request()
        self.state = State.IDLE

    def send_move_above_request(self):
        """Build the desired IkGoalRqstMsg to be sent over the client to make the robot plan and
        execute a trajectory. This request is the trajectory plan for moving above the object."
        """
        # self.request.start_pos.position and orientation already set as last position by API
        self.request.goal_pos.position = [0.5, 0.5, 0.4] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        # self.future contains the plan request
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_move_down_request(self):
        """Generate the trajectory plan for moving down to eventually grip the object.
        """
        self.request.goal_pos.position = [0.5, 0.5, 0.2] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_move_up_request(self):
        """Build the desired IkGoalRqstMsg to be sent over the client to make the robot plan and
        execute a trajectory.
        """
        self.request.goal_pos.position = [0.5, 0.5, 0.4] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_move_home_request(self):
        """Generate the trajectory plan for moving down to eventually grip the object.
        """
        self.request.goal_pos.position = [0.3, 0.0, 0.5] # placeholder values, replace with CV
        self.request.goal_pos.orientation = []
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.cart_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_execute_request(self):
        self.future = self.execute_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    trajectory_client = TrajectoryCaller()
    trajectory_client.send_move_above_request()
    trajectory_client.send_execute_request()
    trajectory_client.send_move_down_request()
    trajectory_client.send_execute_request()
    trajectory_client.send_move_up_request()
    trajectory_client.send_execute_request()
    trajectory_client.send_move_home_request()
    trajectory_client.send_execute_request()
    trajectory_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()