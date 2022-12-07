import rclpy
from rclpy.node import Node
from enum import Enum, auto
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty
import time
from movebot_interfaces.srv import AddBox, GetPlanRqst
from geometry_msgs.msg import Pose

# from moveBot.gripper import Gripper
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp
from franka_msgs.action import Homing
from math import pi

from types import SimpleNamespace


class State(Enum):
    """Create a state machine to eventually implement planning the entire
    stored trajectory plan sequence, for executing it only once at the end.
    """

    IDLE = auto(),
    GO_SCOOP = auto(),
    PLAN = auto(),
    EXECUTE = auto()


class BoxCaller(Node):
    """Spawn in box objects for the planning scene."""

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
        self.state = State.IDLE

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
        self.request = GetPlanRqst.Request()

        # Initialize gripper action clients
        self._gripper_action_client = ActionClient(
            self, GripperCommand, "/panda_gripper/gripper_action"
        )
        self._grasp_client = ActionClient(self, Grasp, "/panda_gripper/grasp")
        self._homing_client = ActionClient(self, Homing, "/panda_gripper/homing")

        # Initialize the timer callback
        self.timer = self.create_timer(1 / 100, self.timer_callback)

        # Define the initial state of the robot
        self.home_x = 0.3
        self.home_y = 0.0
        self.home_z = 0.5
        self.home_roll = 0.0
        self.home_pitch = 0.0
        self.home_yaw = 0.78
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
        self.get_logger().info("grasping...")
        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.speed = speed
        goal_msg.force = force
        goal_msg.epsilon.inner = epsilon[0]
        goal_msg.epsilon.outer = epsilon[1]
        self._grasp_client.wait_for_server()
        return self._grasp_client.send_goal_async(goal_msg)

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
        return self._gripper_action_client.send_goal_async(goal_msg)

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

    # def plan(self, waypoint):
        # """
        # Moves the end-effector the specified waypoint.
        # """
        # assert len(waypoint) == 2, "Invalid waypoint recieved"
        # self.request.goal_pos.position = waypoint[0]
        # self.request.goal_pos.orientation = waypoint[1]
        # self.request.is_xyzrpy = True
        # self.request.execute_now = True
        # self.future = self.cart_client.call_async(self.request)
        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()
        
    def plan(self,waypoint,execute_now=False):
        '''
        Moves the end-effector the specified waypoint
        '''
        assert(len(waypoint) == 2),'Invalid waypoint recieved'
        self.request.goal_pos.position = waypoint[0]
        self.request.goal_pos.orientation = waypoint[1]
        self.request.is_xyzrpy = True
        self.request.execute_now = False
        self.future = self.plan_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        
        # TODO: Need to adjust state machine in API 
        # so we can just set execute_now flag instead of this
        if execute_now:
            self.send_execute_request()

        return self.future.result()

    def send_execute_request(self):
        """
        Execute the trajectory plan - used in each step of the entire
        trajectory sequence.
        """
        self.future = self.execute_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, self.future)
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
                # "scoop_standoff": [
                #     [self.scoop_pose.position.x+0.025,
                #      self.scoop_pose.position.y+0.015,
                #      self.scoop_pose.position.z+0.11],
                #     []
                # ],
                # "scoop_handle": [
                #     [self.scoop_pose.position.x+0.025,
                #      self.scoop_pose.position.y+0.015,
                #      self.scoop_pose.position.z-0.06],
                #     []
                # ],
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
                    self.jig_pose.position.x+0.11,
                    self.jig_pose.position.y+0.08,
                    self.jig_pose.position.z + 0.4, #0.4
                ],
                [],
            ],
            "kettle_return": [
                [
                    self.jig_pose.position.x+0.11,
                    self.jig_pose.position.y+0.08,
                    self.jig_pose.position.z + 0.175,
                ],
                [],
            ],
            "move_test": [[0.3, 0.3, 0.3], []],
            "move_home": [[self.home_x, self.home_y, self.home_z], []],
            "rotate_home": [[], [pi, 0.0, 0.0]],
            "rotate_90": [
                    [],
                    [pi,0.0,pi/2]
                ],
            "kettle_switch_standoff": [
                [
                    self.switch_pose.position.x,
                    self.switch_pose.position.y,
                    self.switch_pose.position.z + 0.1,
                ],
                [],
            ],
            
            "rotate_45": [
                    [],
                    [pi,-pi/4,pi/2]
                ],
            "pour_rot_1": [
                    [],
                    [pi,pi/4,0.0]
                ],
            "pour_so_1": [
                [
                    self.cup_pose.position.x-0.03,
                    self.cup_pose.position.y+0.01,
                    self.cup_pose.position.z + 0.40,
                ],
                [],
            ],
            "pour_rot_2": [
                    [],
                    [pi,pi/2,0.0]
                ],
            "pour_so_2": [
                [
                    self.cup_pose.position.x, #-0.03,
                    self.cup_pose.position.y,#-0.02,
                    self.cup_pose.position.z + 0.4,
                ],
                [],
            ],
            "rotate_45_again": [
                    [],
                    [pi,pi/2.7,pi/2]
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
                    self.stir_pose.position.z + 0.11,
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
                    self.cup_pose.position.z + 0.6,
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

            # "stir_handle": [
            #     [self.stir_pose.position.x,self.stir_pose.position.y,self.stir_pose.position.z+0.11],
            #     []
            # ],
            "cup_tilt_standoff": [
                [self.cup_pose.position.x,self.cup_pose.position.y-0.04,self.cup_pose.position.z+0.2],
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

    def timer_callback(self):
        # Add the collision boxes into the simulation
        # box_client = BoxCaller()
        # box_client.add_box_request()
        # box_client.call_box_request()
        # box_client.add_box2_request()
        # box_client.call_box_request()

        # Update waypoints
        self.define_waypoints()

        # Execute the trajectory sequence
        if self.waypoints is not None:
            # # KETTLE SWITCH
            # # Move the kettle over the cup and pour, then put kettle back
            # self.plan(self.waypoints.kettle_switch_standoff)
            # self.send_execute_request()
            # # Close the gripper before pressing down the switch
            # if not self.GRIP:
            #     self.grasp(width=0.008,force=90.0)
            #     self.GRIP = True
            # self.plan(self.waypoints.kettle_switch)
            # self.send_execute_request()
            # self.plan(self.waypoints.kettle_switch_standoff)
            # self.send_execute_request()
            # if self.GRIP:
            #     self.open_gripper()
            #     self.GRIP = False
            # Add a delay here to allow water to heat up
            # heating_time = 10
            # time.sleep(heating_time)

            # KETTLE
            # Move the kettle over the cup and pour, then put kettle back

            self.plan(self.waypoints.kettle_standoff, execute_now=True)
            self.plan(self.waypoints.kettle, execute_now=True)
            
            if not self.GRIP:
                self.grasp(width=0.008,force=90.0)
                self.GRIP = True
            time.sleep(3)
            self.plan(self.waypoints.kettle_standoff, execute_now=True)

            self.plan(self.waypoints.cup_pour, execute_now=True)
            self.plan(self.waypoints.pour_rot_1, execute_now=True)
            self.plan(self.waypoints.pour_so_1, execute_now=True)
            #self.plan(self.waypoints.pour_so_2, execute_now=True)
            self.plan(self.waypoints.pour_rot_2, execute_now=True)

            self.plan(self.waypoints.pour_rot_1, execute_now=True)
            self.plan(self.waypoints.cup_pour, execute_now=True)
            #self.plan(self.waypoints.pour_so_1, execute_now=True)
            self.plan(self.waypoints.rotate_home, execute_now=True)
            

            # put kettle down
            self.plan(self.waypoints.kettle_return_standoff,execute_now=True)
            self.plan(self.waypoints.kettle_return,execute_now=True)
            self.open_gripper()
            time.sleep(3)
            self.plan(self.waypoints.kettle_return_standoff,execute_now=True)

            # # ROTATE TO POUR
            # self.plan(self.waypoints.kettle_standoff)
            # self.plan(self.waypoints.kettle)
            # if self.GRIP:
            #     self.open_gripper()
            #     self.GRIP = False
            # self.plan(self.waypoints.kettle_standoff)

            # SCOOP
            # Move the cocoa scoop over the cup and pour, then put scoop back
            # self.plan(self.waypoints.rotate_90, execute_now=True)
            # self.plan(self.waypoints.rotate_45,  execute_now=True)
            # self.plan(self.waypoints.scoop_standoff,  execute_now=True)
            # self.plan(self.waypoints.scoop_handle,execute_now=True)
            # if not self.GRIP:
            #     self.grasp(width=0.008,force=90.0)
            #     self.GRIP = True
            # grasp_time = 3
            # time.sleep(grasp_time)
            # self.plan(self.waypoints.scoop_standoff,execute_now=True)
            # self.plan(self.waypoints.cup_tilt_standoff,execute_now=True)
            # self.plan(self.waypoints.rotate_90,execute_now=True)
            # self.plan(self.waypoints.rotate_45_again,execute_now=True)
            # ROTATE TO POUR
            # self.plan(self.waypoints.scoop_standoff)
            # self.plan(self.waypoints.scoop_handle)
            # if self.GRIP:
            #     self.open_gripper()
            #     self.GRIP = False
            #self.plan(self.waypoints.scoop_standoff)

            # # STIR
            # self.plan(self.waypoints.stir_standoff)
            # self.plan(self.waypoints.stir_handle)
            # if not self.GRIP:
            #     self.grasp(width=0.008,force=90.0)
            #     self.GRIP = True
            # self.plan(self.waypoints.stir_standoff)
            # self.plan(self.waypoints.cup_standoff)
            # self.plan(self.waypoints.cup_center)
            # self.stir_duration = 4
            # for i in range(self.stir_duration):
            #     self.plan(self.waypoints.stir1)
            #     self.plan(self.waypoints.stir2)
            #     self.plan(self.waypoints.stir3)
            #     self.plan(self.waypoints.stir4)
            # self.plan(self.waypoints.cup_center)
            # self.plan(self.waypoints.cup_standoff)
            # self.plan(self.waypoints.stir_standoff)
            # self.plan(self.waypoints.stir_handle)
            # if self.GRIP:
            #     self.open_gripper()
            #     self.GRIP = False
            # self.plan(self.waypoints.stir_standoff)

            # Could optionally add a TF for the cup handle to serve the user

            # RETURN HOME
            #self.plan(self.waypoints.move_home)

        # Clear the simulation collision boxes
        #box_client.clear_box_request()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryCaller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()