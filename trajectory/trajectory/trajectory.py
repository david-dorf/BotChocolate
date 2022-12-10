import rclpy
from rclpy.node import Node
from enum import Enum, auto
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty
import time
from movebot_interfaces.srv import AddBox, GetPlanRqst
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp
from franka_msgs.action import Homing
from math import pi
from types import SimpleNamespace

class BoxCaller(Node):
    """Spawn in collision box objects for the planning scene."""
    def __init__(self):
        super().__init__("box_node")
        self.cbgroup = ReentrantCallbackGroup()
        self.add_box_client = self.create_client(
            AddBox, "add_box", callback_group=self.cbgroup
        )
        self.call_box_client = self.create_client(
            Empty, "call_box", callback_group=self.cbgroup
        )
        self.clear_box_client = self.create_client(
            Empty, "clear_all_box", callback_group=self.cbgroup
        )
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
        self.request.z = -0.19 - 0.1  # minus 0.1 because of extra collision height
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


class TrajectoryCaller(Node):
    """Call the plan and execute services from simple_move."""

    def __init__(self):
        super().__init__("trajectory_node")
        self.cbgroup = ReentrantCallbackGroup()

        # Subscribe to object positions from computer vision
        self.scoop_sub = self.create_subscription(
            Pose, "scoop_xzy", self.get_pose_callback, 10
        )
        self.stir_sub = self.create_subscription(
            Pose, "stirrer_xyz", self.get_stir_pose_callback, 10
        )
        self.cup_sub = self.create_subscription(
            Pose, "cup_xyz", self.get_cup_pose_callback, 10
        )
        self.kettle_sub = self.create_subscription(
            Pose, "kettle_xyz", self.get_kettle_pose_callback, 10
        )
        self.kettle_switch_sub = self.create_subscription(
            Pose, "kettle_switch_xyz", self.get_kettle_switch_pose_callback, 10
        )
        self.jig_sub = self.create_subscription(
            Pose, "jig_xyz", self.get_jig_pose_callback, 10
        )

        # Initialize robot trajectory clients
        self.plan_client = self.create_client(
            GetPlanRqst, "call_plan", callback_group=self.cbgroup
        )
        self.cart_client = self.create_client(
            GetPlanRqst, "call_cart", callback_group=self.cbgroup
        )
        self.execute_client = self.create_client(
            Empty, "call_execute", callback_group=self.cbgroup
        )

        self.execute_final_path_client = self.create_service(
            Empty, "make_hot_chocolate", self.make_chocolate_callback
        )

        self.request = GetPlanRqst.Request()

        # Initialize gripper action clients
        self._gripper_action_client = ActionClient(
            self, GripperCommand, "/panda_gripper/gripper_action"
        )
        self._grasp_client = ActionClient(self, Grasp, "/panda_gripper/grasp")
        self._homing_client = ActionClient(self, Homing, "/panda_gripper/homing")

        self.plan_scene_flag=0
        self.trial_flag=0

        # Define the initial state of the robot
        self.home_x = 0.3
        self.home_y = 0.0
        self.home_z = 0.5
        self.home_roll = pi
        self.home_pitch = 0.0
        self.home_yaw = 0.0
        self.GRIP = False
        
    def get_kettle_pose_callback(self, pose_msg):
        self.kettle_pose = pose_msg

    def get_pose_callback(self, pose_msg):
        self.scoop_pose = pose_msg

    def get_stir_pose_callback(self, pose_msg):
        self.stir_pose = pose_msg

    def get_cup_pose_callback(self, pose_msg):
        self.cup_pose = pose_msg

    def get_kettle_switch_pose_callback(self, pose_msg):
        self.switch_pose = pose_msg
    
    def get_jig_pose_callback(self, pose_msg):
        self.jig_pose = pose_msg

    def grasp(self, width, speed=1.0, force=30.0, epsilon=(0.005, 0.005)):
        """
        Grasps an object. It can fail if the width is not accurate

        :param width: width of the object you are grasping
        :param speed: speed the gripper will close
        :param force: force the gripper will grasp the object
        :param epsilon: inner and outer tolerance
        """
        self.get_logger().info("Grasping...")
        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.speed = speed
        goal_msg.force = force
        goal_msg.epsilon.inner = epsilon[0]
        goal_msg.epsilon.outer = epsilon[1]
        self._grasp_client.wait_for_server()
        self.future_grasp_res= self._grasp_client.send_goal_async(goal_msg)
        self.get_logger().info("Done Grasping")
        return self.future_grasp_res

    def open_gripper(self):
        """
        Opens the gripper, position=0.04 is open for some reason

        :return: A future object from the ActionClient.send_goal_async()
        function
        """
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.04
        goal_msg.command.max_effort = 1.0
        self._gripper_action_client.wait_for_server()
        self.future_open_res= self._gripper_action_client.send_goal_async(goal_msg)
        self.get_logger().info("Gripper Open")
        return self.future_open_res

    def home_gripper(self):
        """
        Homes the gripper by first closing the gripper, then opening all the
        way.

        :return: A future object from the ActionClient.send_goal_async()
        function
        """
        goal_msg = Homing.Goal()
        self._homing_client.wait_for_server()
        return self._homing_client.send_goal_async(goal_msg)

        
    def plan(self,waypoint,execute_now):
        """
        Create a trajectory plan for the end-effector to reach thespecified waypoint

        :return: If in x,y,z,r,p,y configuration, a cartesian trajectory plan. If otherwise,
        a rotation motion plan.
        """
        assert(len(waypoint) == 2),'Invalid waypoint recieved'
        self.request.goal_pos.position = waypoint[0]
        self.request.goal_pos.orientation = waypoint[1]

        if len(waypoint[0])>3:
            self.request.is_xyzrpy = False  # SENDING HOME JOINT STATES
        else:
            self.request.is_xyzrpy = True
        self.request.execute_now = False

        self.future = self.plan_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        if execute_now and not self.request.goal_pos.orientation :
            self.send_execute_request()
        elif execute_now and not self.request.goal_pos.position:
            self.get_logger().info(f" FINISHED EXECUTING- CHANGING STATES")
            self.send_execute_request()
        return self.future.result()

    def send_execute_request(self):
        """
        Execute the trajectory plan - used in each step of the entire
        trajectory sequence.

        :return: Empty
        """
        self.future = self.execute_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, self.future)

        self.get_logger().info(f" FINISHED EXECUTING- CHANGING STATES")

        return self.future.result()

    def define_waypoints(self):
        """
        Creates a dictionary of waypoints based on the current TF data
        for the different objects. Should be called in timer loop to
        get up to date data.

        Each waypoint must be a nested list of length 2. The first element
        is a list of length 3 corresponding to the waypoint (x,y,z) and the
        second element is another list of length 2 corresponding to the
        (roll, pitch, yaw) of the EE at the waypoint.
        """
        waypoints_dict = {

            "send_home": [

                    [-0.001312,-0.787102,-0.003124,-2.357184,-0.000188,1.571918,0.783990, 0.000323, 0.000323],
                    []
                ],
            "new_home": [

                    [pi/2,-0.783508,-0.000244,-2.356829,-0.003843,1.572944,-0.784056, 0.034865, 0.034865],
                    []
                ],
            "scoop_standoff": [
                    [self.scoop_pose.position.x,
                     self.scoop_pose.position.y,
                     self.scoop_pose.position.z+0.15],
                    []
                ],
            "scoop_handle": [
                    [self.scoop_pose.position.x,
                     self.scoop_pose.position.y,
                     self.scoop_pose.position.z],
                    []
                ],
            "kettle_standoff": [
                [
                    self.kettle_pose.position.x,
                    self.kettle_pose.position.y,
                    self.kettle_pose.position.z + 0.15,
                ],
                [],
            ],
            "kettle": [
                [
                    self.kettle_pose.position.x,
                    self.kettle_pose.position.y,
                    self.kettle_pose.position.z + 0.035,
                ],
                [],
            ],
            "kettle_return_standoff": [
                [
                    self.jig_pose.position.x-0.3,
                    self.jig_pose.position.y-0.02,
                    self.jig_pose.position.z + 0.4, #0.4
                ],
                [],
            ],
            "kettle_return": [
                [
                    self.jig_pose.position.x-0.3,
                    self.jig_pose.position.y-0.02,
                    self.jig_pose.position.z + 0.13,
                ],
                [],
            ],
            "move_test": [[0.3, 0.3, 0.3], []],
            "move_home": [[self.home_x, self.home_y, self.home_z], []],
            "rot_home":[[], [self.home_roll, self.home_pitch, self.home_yaw]],
            "rotate_home": [[], [pi, 0.0, 0.0]],
            "rotate_90": [
                    [],
                    [pi,0.0,pi/2]
                ],
            "straighten":[
                [],
                [pi,0.0,0.0]
                ],
            "kettle_switch_standoff": [
                [
                    self.switch_pose.position.x,
                    self.switch_pose.position.y,
                    self.switch_pose.position.z + 0.1,
                ],
                [],
            ],
            "rotate_minus_45_yaw": [
                    [],
                    [pi,0.0,-pi/4]
                ],
            "rotate_minus_0_yaw": [
                    [],
                    [pi,0.0,0.0]
                ],
            "rotate_45": [
                    [],
                    [pi,-pi/5,pi/2]
                ],
            "rotate_45_back": [
                    [],
                    [pi,-pi/4,pi/2]
                ],
            "pour_rot_1": [
                    [],
                    [pi,0.0,pi]
                ],
            "pour_rot_2": [
                    [],
                    [pi,-pi/4,pi]
                ],
            "pour_rot_3": [
                    [],
                    [pi,-pi/2,pi]
                ],
            "pour_so_1": [
                [
                    self.cup_pose.position.x-0.03,
                    self.cup_pose.position.y+0.01,
                    self.cup_pose.position.z + 0.45,
                ],
                [],
            ],
            "pour_so_2": [
                [
                    self.cup_pose.position.x -0.03,
                    self.cup_pose.position.y,
                    self.cup_pose.position.z + 0.38 ,
                ],
                [],
            ],
            "rotate_45_again": [
                    [],
                    [pi,pi/2.7,pi/2]
                ],
            
            "pour_rot_final": [
                    [],
                    [pi,-0.1,pi]
                ],
            
            
            "kettle_switch": [
                [
                    self.switch_pose.position.x,
                    self.switch_pose.position.y,
                    self.switch_pose.position.z - 0.005,
                ],
                [],
            ],
            "stir_standoff": [
                [
                    self.stir_pose.position.x,
                    self.stir_pose.position.y,
                    self.stir_pose.position.z + 0.3,
                ],
                [],
            ],
            "stir_handle": [
                [
                    self.stir_pose.position.x,
                    self.stir_pose.position.y,
                    self.stir_pose.position.z + 0.05,
                ],
                [],
            ],
            "cup_standoff": [
                [
                    self.cup_pose.position.x,
                    self.cup_pose.position.y,
                    self.cup_pose.position.z + 0.3,
                ],
                [],
            ],

            "cup_pour": [
                [
                    self.cup_pose.position.x + -0.1,
                    self.cup_pose.position.y,
                    self.cup_pose.position.z + 0.5,
                ],
                [],
            ],
            "cup_center": [
                [
                    self.cup_pose.position.x,
                    self.cup_pose.position.y,
                    self.cup_pose.position.z + 0.1,
                ],
                [],
            ],

            "stir1": [
                [
                    self.cup_pose.position.x + 0.02,
                    self.cup_pose.position.y,
                    self.cup_pose.position.z + 0.1,
                ],
                [],
            ],
            "cup_tilt_standoff": [
                [self.cup_pose.position.x,self.cup_pose.position.y-0.02,self.cup_pose.position.z+0.2],
                []
            ],
            
            "cup_standoff": [
                [self.cup_pose.position.x,self.cup_pose.position.y,self.cup_pose.position.z+0.3],
                []
            ],

            "cup_handle": [
                [self.cup_pose.position.x,self.cup_pose.position.y,self.cup_pose.position.z+0.3],
                []
            ],
            "stir2": [
                [
                    self.cup_pose.position.x,
                    self.cup_pose.position.y + 0.02,
                    self.cup_pose.position.z + 0.1,
                ],
                [],
            ],
            "stir3": [
                [
                    self.cup_pose.position.x - 0.02,
                    self.cup_pose.position.y,
                    self.cup_pose.position.z + 0.1,
                ],
                [],
            ],
            "stir4": [
                [
                    self.cup_pose.position.x,
                    self.cup_pose.position.y - 0.02,
                    self.cup_pose.position.z + 0.1,
                ],
                [],
            ],
        }
        self.waypoints = SimpleNamespace(**waypoints_dict)

    def make_chocolate_callback(self,request,response):
        """
        Summons in collision objects into the planning scene and executes the trajectory sequence
        to create hot chocolate.
        """
        # Call in the collision boxes into the planning scene
        if self.plan_scene_flag==0:
            box_client = BoxCaller()
            box_client.add_box_request()
            box_client.call_box_request()
            box_client.add_box2_request()
            box_client.call_box_request()
            self.plan_scene_flag+=1
        # Call in the waypoints from the dictionary
        self.define_waypoints()
        # Kettle switch
        self.plan(self.waypoints.new_home, execute_now=True)
        self.plan(self.waypoints.kettle_switch_standoff, execute_now=True)
        if not self.GRIP:
            self.grasp(width=0.008,force=90.0)
            self.GRIP = True
        time.sleep(3)
        self.GRIP = False
        self.plan(self.waypoints.kettle_switch, execute_now=True)
        self.plan(self.waypoints.kettle_switch_standoff, execute_now=True)
        self.open_gripper()
        time.sleep(3)
        # Cocoa scoop
        self.plan(self.waypoints.cup_tilt_standoff, execute_now=True)
        self.plan(self.waypoints.new_home, execute_now=True)
        self.plan(self.waypoints.scoop_standoff, execute_now=True)
        self.plan(self.waypoints.rotate_90, execute_now=True)
        self.plan(self.waypoints.rotate_45, execute_now=True)
        self.plan(self.waypoints.scoop_handle, execute_now=True)
        self.grasp(width=0.008,force=90.0)
        time.sleep(3)
        self.plan(self.waypoints.scoop_standoff, execute_now=True)
        self.plan(self.waypoints.cup_tilt_standoff, execute_now=True)
        self.plan(self.waypoints.rotate_90, execute_now=True)
        self.plan(self.waypoints.rotate_45_again, execute_now=True)
        self.plan(self.waypoints.rotate_90, execute_now=True)
        self.plan(self.waypoints.cup_tilt_standoff, execute_now=True)
        self.plan(self.waypoints.scoop_standoff, execute_now=True)
        self.plan(self.waypoints.rotate_45, execute_now=True)
        self.plan(self.waypoints.scoop_handle, execute_now=True)
        self.open_gripper()
        time.sleep(3)
        self.plan(self.waypoints.scoop_standoff, execute_now=True)
        self.plan(self.waypoints.new_home, execute_now=True)
        # Kettle pour
        self.plan(self.waypoints.kettle_standoff, execute_now=True)
        self.plan(self.waypoints.kettle, execute_now=True)
        self.grasp(width=0.008,force=90.0)
        time.sleep(3)
        self.plan(self.waypoints.kettle_standoff, execute_now=True)
        self.plan(self.waypoints.cup_pour, execute_now=True)
        self.plan(self.waypoints.pour_so_1, execute_now=True)
        self.plan(self.waypoints.pour_rot_1, execute_now=True)
        self.plan(self.waypoints.pour_rot_2, execute_now=True)
        self.plan(self.waypoints.pour_so_2, execute_now=True)
        self.plan(self.waypoints.pour_rot_3, execute_now=True)
        time.sleep(5)
        self.plan(self.waypoints.pour_so_1, execute_now=True)
        self.plan(self.waypoints.pour_rot_2, execute_now=True)
        self.plan(self.waypoints.pour_rot_1, execute_now=True)
        self.plan(self.waypoints.kettle_return_standoff,execute_now=True)
        self.plan(self.waypoints.pour_rot_1, execute_now=True)
        self.plan(self.waypoints.kettle_return,execute_now=True)
        self.open_gripper()
        time.sleep(3)
        self.plan(self.waypoints.kettle_return_standoff,execute_now=True)
        # Stir
        self.plan(self.waypoints.stir_standoff, execute_now=True)
        self.plan(self.waypoints.stir_handle, execute_now=True)
        if not self.GRIP:
            self.grasp(width=0.001,force=90.0)
            self.GRIP = True
        time.sleep(2)
        self.plan(self.waypoints.stir_standoff, execute_now=True)
        self.plan(self.waypoints.cup_standoff, execute_now=True)
        self.plan(self.waypoints.cup_center, execute_now=True)
        self.stir_duration = 4
        for i in range(self.stir_duration):
            self.plan(self.waypoints.stir1, execute_now=True)
            self.plan(self.waypoints.stir2, execute_now=True)
            self.plan(self.waypoints.stir3, execute_now=True)
            self.plan(self.waypoints.stir4, execute_now=True)
        self.plan(self.waypoints.cup_center, execute_now=True)
        self.plan(self.waypoints.cup_standoff, execute_now=True)
        self.plan(self.waypoints.stir_standoff, execute_now=True)
        self.plan(self.waypoints.stir_handle, execute_now=True)
        if self.GRIP:
            self.open_gripper()
            self.GRIP = False
        self.plan(self.waypoints.stir_standoff, execute_now=True)
        self.plan(self.waypoints.cup_tilt_standoff, execute_now=True)
        # Return home
        self.plan(self.waypoints.new_home, execute_now=True)
        self.trial_flag+=1
        # Delete boxes
        box_client.clear_box_request()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryCaller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()