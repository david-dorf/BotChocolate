import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from moveit_msgs.action import MoveGroup, ExecuteTrajectory

class Intercept(Node):
    def __init__(self):
        super().__init__("fake_movegroup")
        self._action_server = ActionServer(
            self, 
            MoveGroup,
            "move_action",
            self.move_action_callback)

        self._action_server = ActionServer(
            self, 
            ExecuteTrajectory,
            "execute_trajectory",
            self.et_callback)
        
        
    def move_action_callback(self, goal):
        self.get_logger().info(f"move{goal.request.request}")
        result = MoveGroup.Result()
        return result
    
    def et_callback(self, goal):
        self.get_logger().info(f"et{goal.request}")
        result = ExecuteTrajectory.Result()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = Intercept()
    rclpy.spin(node)
    rclpy.shutdown()