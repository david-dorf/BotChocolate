# from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
import rclpy


class MoveBot():
    def __init__(self, node):
        self.node = node
        # TODO make this client so that we can give compute_ik action a  position and get joint
        # positions
        self._ik_action_client = ActionClient(
            self,
            MoveGroup,
            "move_action",
            self.move_action_callback)
        self.ik_service = self.create_client(GetPositionIK, "/compute_ik")

    def create_path(self, x_goal, y_goal, z_goal, r_goal, p_goal, yaw_goal):
        """
        Take in x,y,z,r,p,y pose of robot and creates a path to goal.

        Args:
        ----
            x_goal (_type_): _description_
            y_goal (_type_): _description_
            z_goal (_type_): _description_
            r_goal (_type_): _description_
            p_goal (_type_): _description_
            yaw_goal (_type_): _description_

        """
        # Take in position and put in se(3)
        # Call compute_ik -> gives joints positions of goal
        # Helper functoin get_joint_positions

    def get_joint_positions(self, x_goal, y_goal, z_goal, r_goal, p_goal, yaw_goal):
        # Call comput ik service
        position = GetPositionIK()
        position.pose.position.x = 0.0

        # self.ik_service.call_async(GetPositionIK.Request(ik_request.pose.position.x = 0.0,
        #                                                  = xx , y = yy, theta = 0.0))
    # Call moveit action
    # Execute


def main(args=None):
    rclpy.init(args=args)
    node = MoveBot()
    rclpy.spin(node)
    rclpy.shutdown()
