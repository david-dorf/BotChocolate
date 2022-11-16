import rclpy
from rclpy.node import Node
from enum import Enum, auto
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty
from movebot_interfaces.srv import IkGoalRqst, AddBox, GetPlanRqst
from movebot_interfaces.msg import IkGoalRqstMsg

# class State(Enum):
#     OFF = auto(),
#     MOVEMENT1 = auto(),
#     MOVEMENT2 = auto()

class TrajectoryCaller(Node):
    """Call the plan and execute services from simple_move."""
    def __init__(self):
        super().__init__("trajectory_node")
        self.cbgroup = ReentrantCallbackGroup()
        self.plan_client = self.create_client(GetPlanRqst,"call_plan",callback_group=self.cbgroup)
        self.execute_client = self.create_client(Empty,"call_execute",callback_group=self.cbgroup)
        self.request = GetPlanRqst.Request()

    async def send_request(self):
        """Build the desired IkGoalRqstMsg to be sent over the client to make the robot plan and
        execute a trajectory.
        """
        self.request.start_pos = IkGoalRqstMsg()
        self.request.goal_pos = IkGoalRqstMsg()
        self.request.start_pos.position = []
        self.request.start_pos.orientation = []
        self.request.goal_pos.position = [0.5, 0.5, 0.5] # placeholder values, replace with CV
        self.request.goal_pos.orientation = [0.0, 0.0, 0.0]
        # find gripper flag in interface proto for GetPlanRqst
        response = await self.plan_client.call_async(self.request)
        return response


def main(args=None):
    rclpy.init(args=args)
    trajectory_client = TrajectoryCaller()
    trajectory_client.send_request()
    # rclpy.spin(trajectory_client)
    trajectory_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
