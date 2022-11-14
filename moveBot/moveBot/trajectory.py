import rclpy
from rclpy.node import Node
from enum import Enum, auto
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty
from movebot_interfaces.srv import IkGoalRqst, GetPlanRqst, AddBox
from movebot_interfaces.msg import IkGoalRqstMsg

# class State(Enum):
#     OFF = auto(),
#     MOVEMENT1 = auto(),
#     MOVEMENT2 = auto()

class TrajectoryCaller(Node):
    def __init__(self):
        super().__init__("trajectory_node")
        self.cbgroup = ReentrantCallbackGroup()
        self.plan_client = self.create_client(GetPlanRqst,"call_plan")
        self.execute_client = self.create_client(Empty,"call_execute")

    async def send_request(self, request, response):
        request = IkGoalRqstMsg()
        request.position = [0.5, 0.5, 0.5] # placeholder values, replace with CV
        request.orientation = [0.0, 0.0, 0.0]
        # find gripper flag in interface proto for GetPlanRqst
        response = await self.plan_client.call_async(request)
        return response


def main(args=None):
    rclpy.init(args=args)
    trajectory_client = TrajectoryCaller()
    # trajectory_client.send_request()
    trajectory_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
