import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    PositionIKRequest,
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    RobotState,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPlanningScene, GetCartesianPath
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from shape_msgs.msg import SolidPrimitive
import math
import numpy as np
from movebot_interfaces.srv import IkGoalRqst, GetPlanRqst, AddBox
from movebot_interfaces.msg import IkGoalRqstMsg, SimpleCartPath


from enum import Enum, auto


def quaternion_from_euler(ai, aj, ak):
    """
    Take in Euler angles and converts them to quaternions. Function taken from link above.

    :param ai: Roll angle.
    :type ai: float
    :param aj: Pitch angle.
    :type aj: float
    :param ak: Yaw angle.
    :type ak: float

    :return: Array of quaternion angles.
    :rtype: ndarray

    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


class State(Enum):
    IDLE = (auto(),)
    ROT_MSG = (auto(),)
    CART_MSG = (auto(),)
    HOME_MSG = (auto(),)
    CART_EXEC = (auto(),)
    PLAN_EXEC = auto()


class MoveBot(Node):
    """
    Turn a goal end effector position and orientation into a trajectory to be
    executed by the Franka Emika robot arm. The position and orientation of the
    end effector is given in relation to the frame located at the base of the robot.

    """

    def __init__(self):
        super().__init__("simple_move")
        self.cbgroup = ReentrantCallbackGroup()
        self._plan_client = ActionClient(
            self,
            MoveGroup,
            "move_action",
        )

        self._execute_client = ActionClient(
            self, ExecuteTrajectory, "execute_trajectory"
        )

        self.jointpub = self.create_subscription(
            JointState, "/joint_states", self.js_cb, 10
        )

        self.ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self.cbgroup
        )

        self.call_ik = self.create_service(
            IkGoalRqst, "call_ik", self.ik_callback, callback_group=self.cbgroup
        )

        self.cart_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path", callback_group=self.cbgroup
        )

        self.call_cart = self.create_service(
            GetPlanRqst, "call_cart", self.plan_callback, callback_group=self.cbgroup
        )

        self.call_plan = self.create_service(
            GetPlanRqst, "call_plan", self.plan_callback, callback_group=self.cbgroup
        )

        self.call_execute = self.create_service(
            Empty, "call_execute", self.execute_callback, callback_group=self.cbgroup
        )

        self.timer = self.create_timer(1 / 100, self.timer_callback)
        self.box_publisher = self.create_publisher(PlanningScene, "planning_scene", 10)

        self.call_box = self.create_service(
            Empty, "call_box", self.box_callback, callback_group=self.cbgroup
        )

        self.clear_all_box = self.create_service(
            Empty, "clear_all_box", self.clear_callback, callback_group=self.cbgroup
        )

        self.clear_current_box = self.create_service(
            Empty,
            "clear_current_box",
            self.remove_callback,
            callback_group=self.cbgroup)

        self.scene_client = self.create_client(
            GetPlanningScene, "get_planning_scene", callback_group=self.cbgroup
        )
        self.update_box = self.create_service(
            AddBox, "add_box", self.update_box_callback
        )

        self.box_x = 0.2
        self.box_y = 0.2
        self.box_z = 0.2
        self.box_l = 0.2
        self.box_w = 0.2
        self.box_h = 0.2
        self.box_name = "box_0"
        self.single_joint_rotation_cnt = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.time = 0
        self.state = State.IDLE

    def update_box_callback(self, request, response):
        """
        Callback function for add_box service.

        Update the box information
        (id, position, size)

        :param request: Box information
        :type request: movebot_interface/srv/AddBox.request
        :param response: Empty
        :type response: movebot_interface/srv/AddBox.response

        :returns: Empty

        """
        self.box_x = request.x
        self.box_y = request.y
        self.box_z = request.z
        self.box_l = request.l
        self.box_w = request.w
        self.box_h = request.h
        self.box_name = str(request.name)
        return response

    async def box_callback(self, request, response):
        """
        Callback function for call_box service.

        Publish saved box information to Planning Scene.
        Get current Planning Scene from get_planning_scene.
        Add/Update box in the world.collision_objects.
        Publish updated Planning Scene.

        :param request: Empty
        :type request: movebot_interface/srv/AddBox.request
        :param response: Empty
        :type response: movebot_interface/srv/AddBox.response

        :rtype: std_srvs.srv.Empty.Response()

        """
        component = PlanningSceneComponents()
        Scene_raw = await self.scene_client.call_async(
            GetPlanningScene.Request(components=component)
        )
        Scene = Scene_raw.scene
        box = CollisionObject()
        box.header.stamp = self.get_clock().now().to_msg()
        box.header.frame_id = "panda_link0"
        box.pose.position.x = self.box_x
        box.pose.position.y = self.box_y
        box.pose.position.z = self.box_z
        box.pose.orientation.x = 0.0
        box.pose.orientation.y = 0.0
        box.pose.orientation.z = 0.0
        box.pose.orientation.w = 1.0
        box.id = self.box_name
        SP = SolidPrimitive()
        SP.type = 1
        SP.dimensions = [self.box_l, self.box_w, self.box_h]
        box.primitives = [SP]

        SPPose = Pose()
        box.primitive_poses = [SPPose]

        exist = 0
        for i in Scene.world.collision_objects:
            if i.id == self.box_name:
                i.pose.position.x = self.box_x
                i.pose.position.y = self.box_y
                i.pose.position.z = self.box_z
                i.primitives = [SP]
                exist = 1
                break
        if exist == 0:
            Scene.world.collision_objects.append(box)
        self.box_publisher.publish(Scene)

        return Empty.Response()

    async def clear_callback(self, request, response):
        """
        Call back fucntion for clear_all_box service.

        Clear all collision_objects in PlanningScene.world

        :param request: Empty.
        :type request: movebot_interface/srv/AddBox.request
        :param response: Empty
        :type response: movebot_interface/srv/AddBox.response

        :rtype: std_srvs.srv.Empty.Response()

        """
        component = PlanningSceneComponents()
        Scene_raw = await self.scene_client.call_async(
            GetPlanningScene.Request(components=component)
        )
        Scene = Scene_raw.scene

        Scene.world.collision_objects = []

        self.box_publisher.publish(Scene)

        return Empty.Response()

    # async def remove_callback(self, request, response):
    #     """
    #     Call back function for clear_current_box service.

    #     Clear the box in PlanningScene.world.collision_objects
    #     with the id which is match the save box information

    #     :param request: Empty.
    #     :type request: movebot_interface/srv/AddBox.request
    #     :param response: Empty.
    #     :type response: movebot_interface/srv/AddBox.response
    
    #     :rtype: std_srvs.srv.Empty.Response()

    #     """
    #     component = PlanningSceneComponents()
    #     Scene_raw = await self.scene_client.call_async(
    #         GetPlanningScene.Request(components=component)
    #         )
    #     Scene = Scene_raw.scene

    #     for i in Scene.world.collision_objects:
    #         if i.id == self.box_name:
    #             Scene.world.collision_objects.remove(i)
    #             break

    #     self.box_publisher.publish(Scene)

    #     return Empty.Response()

    def js_cb(self, jointstate):
        """
        Call back function of the goal pose subscriber. Stores the GoalPose message received.

        :param jointstate: Contains the information of the current robot joint angles.
        :type joinstate: sensor_msgs/msg/JointState

        """
        self.joint_statesmsg = jointstate

    def get_ik_rqst_msg(self, pose_vec):
        """
        Get IK request message.

        Process a pose request message and turn it into a response usable by the inverse
        kinematics callback function.

        :param pose_vec: Contains the request message to be converted into a usable IK msg.
        :type pose_vec: ndarray

        :return: Processed version of the pose_vec that can be used by IK.
        :rtype: moveit_msgs/msg/PositionIKRequest

        """
        ikmsg = PositionIKRequest()
        ikmsg.group_name = "panda_manipulator"
        ikmsg.robot_state.joint_state = self.joint_statesmsg

        ikmsg.pose_stamped.header.frame_id = "panda_link0"
        ikmsg.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ikmsg.pose_stamped.pose.position.x = pose_vec[0]
        ikmsg.pose_stamped.pose.position.y = pose_vec[1]
        ikmsg.pose_stamped.pose.position.z = pose_vec[2]

        quats = quaternion_from_euler(pose_vec[3], pose_vec[4], pose_vec[5])
        ikmsg.pose_stamped.pose.orientation.x = quats[0]
        ikmsg.pose_stamped.pose.orientation.y = quats[1]
        ikmsg.pose_stamped.pose.orientation.z = quats[2]
        ikmsg.pose_stamped.pose.orientation.w = quats[3]
        ikmsg.timeout.sec = 5

        # self.get_logger().info(f"IK QUATS   {quats}")

        return ikmsg

    async def ik_callback(self, request, response):
        """
        IK Callback Function.

        Generate the inverse kinematics solution that gives the joint angles to reach a desired
        end effector configuration.

        :param request: Request pose of initial location
        :type request: Float64[] position Float64[] orientation
        :param response: Computes the IK solution for the given ikmsg.
        :type response: moveit_msgs/msg/RobotState

        :return: The computed joint angles from the inverse kinematics function.
        :rtype: moveit_msgs/msg/RobotState

        """
        if not request.position and not request.orientation:
            current_position = [
                self.ee_base.transform.translation.x,
                self.ee_base.transform.translation.y,
                self.ee_base.transform.translation.z,
            ]

            current_orientation = [0.0, 0.0, 0.0]
            pose_vec = np.hstack([current_position, current_orientation])
        elif not request.position:
            current_position = [
                self.ee_base.transform.translation.x,
                self.ee_base.transform.translation.y,
                self.ee_base.transform.translation.z,
            ]

            pose_vec = np.hstack([current_position, request.orientation])
        elif not request.orientation:

            current_orientation = [0.0, 0.0, 0.0]

            pose_vec = np.hstack([request.position, current_orientation])
        else:
            pose_vec = np.hstack([request.position, request.orientation])
        # self.get_logger().info(f"goal pose vec {pose_vec}")

        msg = self.get_ik_rqst_msg(pose_vec)

        # self.get_logger().info(f"goal ik req msg {msg}")
        self.ik_response = await self.ik_client.call_async(
            GetPositionIK.Request(ik_request=msg)
        )
        response.joint_state = self.ik_response.solution.joint_state
        # self.get_logger().info(f"goal ik response {response}")

        return response

    def create_cart_msg(self, start, goal):

        cart_req = SimpleCartPath()
        cart_req.cart_request.header.stamp = self.get_clock().now().to_msg()
        cart_req.cart_request.header.frame_id = "panda_link0"
        cart_req.cart_request.start_state.joint_state = start.joint_state
        # print(f"\n START STATE {cart_req.cart_request.start_state.joint_state}")
        

        cart_req.cart_request.group_name = 'panda_manipulator'
        cart_req.cart_request.max_step=0.01
        cart_req.cart_request.jump_threshold= 0.0
        cart_req.cart_request.prismatic_jump_threshold= 20.0
        cart_req.cart_request.revolute_jump_threshold= 20.0 

        wp1=Pose()
        # print(f"\n GP {goal.position[:]}")
        wp1.position.x=goal.position[0]
        wp1.position.y=goal.position[1]
        wp1.position.z=goal.position[2]


        if len(goal.orientation) == 3:
            quats = quaternion_from_euler(
                goal.orientation[0], goal.orientation[1], goal.orientation[2]
            )
        else:
            quats=[goal.orientation[0],goal.orientation[1],goal.orientation[2],goal.orientation[3]]
        # r,p,y=euler_from_quaternion(goal.orientation[0],goal.orientation[1],goal.orientation[2],goal.orientation[3])
        # self.get_logger().info(f"LISTENER X ROT {r}", once=True)
        # self.get_logger().info(f"LISTENER Y ROT {p}", once=True)
        # self.get_logger().info(f"LISTENER Z ROT {y}", once=True)
        
        # print(f"\n OR QUATS {quats}")
        wp1.orientation.x=quats[0]
        wp1.orientation.y=quats[1]
        wp1.orientation.z=quats[2]
        wp1.orientation.w=quats[3]

        # print(f"\n WP1 {wp1}")
        cart_req.cart_request.waypoints= [wp1]

        return cart_req

    def get_motion_request(self, start, goal, execute):
        """

        Process a motion request message into a response usable by the motion planning.

        :param start: Start configuration of the robot.
        :type start: moveit_msgs/msg/RobotState
        :param goal: End goal configuration of the robot.
        :type goal: moveit_msgs/msg/RobotState
        :param execute: Start execute immediately or manually.
        :type execute: bool

        :return: Request to generate the trajectory in the callback.
        :rtype: MoveGroup_Goal

        """
        motion_req = MotionPlanRequest()
        motion_req.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        motion_req.workspace_parameters.min_corner.x = -1.0
        motion_req.workspace_parameters.min_corner.y = -1.0
        motion_req.workspace_parameters.min_corner.z = -1.0
        motion_req.workspace_parameters.max_corner.x = 1.0
        motion_req.workspace_parameters.max_corner.y = 1.0
        motion_req.workspace_parameters.max_corner.z = 1.0
        motion_req.workspace_parameters.header.frame_id = "panda_link0"
        motion_req.start_state.joint_state = start.joint_state
        goal_constraints = Constraints()

        for i in range(len(self.joint_statesmsg.name)):
            joint_constraints = JointConstraint()
            joint_constraints.joint_name = self.joint_statesmsg.name[i]
            joint_constraints.position = goal.joint_state.position[
                i
            ]  # goal.joint_state.position[i]

            joint_constraints.tolerance_above = 0.002
            joint_constraints.tolerance_below = 0.002
            joint_constraints.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraints)
        if goal.joint_state.position == self.joint_statesmsg.position:
            if self.single_joint_rotation_cnt == 0:
                self.get_logger().info("LOGGER CAUSE WE GOING NEW HOME")
                goal_constraints.joint_constraints[0].position = np.pi / 2
                goal_constraints.joint_constraints[6].position = -np.pi / 4
                self.homeconfig = Constraints()
                self.homeconfig = goal_constraints
                self.single_joint_rotation_cnt += 1
            elif self.single_joint_rotation_cnt > 0:
                self.get_logger().info("LOGGER CAUSE WE GOING NEW HOME")
                for i in range(len(self.homeconfig.joint_constraints)):
                    goal_constraints.joint_constraints[
                        i
                    ].position = self.homeconfig.joint_constraints[i].position
                self.single_joint_rotation_cnt += 1
            # elif self.single_joint_rotation_cnt == 2:
            #     goal_constraints.joint_constraints[4].position= np.pi/4
            #     self.single_joint_rotation_cnt +=1
            # elif self.single_joint_rotation_cnt == 3:
            #     goal_constraints.joint_constraints[4].position= np.pi/2
            #     self.single_joint_rotation_cnt +=1
        # self.get_logger().info(f"goal {goal_constraints.joint_constraints[6].position}")

        motion_req.goal_constraints = [goal_constraints]
        motion_req.pipeline_id = "move_group"
        motion_req.group_name = "panda_manipulator"
        motion_req.num_planning_attempts = 100
        motion_req.allowed_planning_time = 10.0

        motion_req.max_velocity_scaling_factor = 0.1
        motion_req.max_acceleration_scaling_factor = 0.1
        motion_req.max_cartesian_speed = 0.0
        plan_request = MoveGroup.Goal()
        plan_request.request = motion_req

        if execute:
            plan_request.planning_options.plan_only = False
        else:
            plan_request.planning_options.plan_only = True
        return plan_request

    async def plan_callback(self, request, response):
        """
        Plan the desired trajectory path that the robot arm will follow.

        This path will be followed by the robot upon execution.

        Depending on the given request, the motion plan message is
        either created with with the get_motion_request or the 
        create_cart_msg. Both of these return a message that are passed 
        as a request to either the MoveGroup action client 
        or GetCartesianPath service client respectively. 

        :param request: Message that includes the start position and goal position of the
        trajectory, and a flag that indicates if its in XYZ coords or joint positions, as well
        as a flag that determines if it should execute immediately or not.
        :type request: GetPlanRqst

        :param response: Message to send to the execute_trajectory action client.
        :type response: ExecuteTrajectory

        """

        self.get_logger().info(f"READY TO PLAN {self.state}")
        if request.is_xyzrpy:  # If start pos was given as X,Y,Z, R, P, Y
            if len(request.start_pos.position) <= 0:
                # IF there is no given start position, use current joint config as start
                request.start_pos.position = self.joint_statesmsg.position
                start_in_joint_config = RobotState()
                start_in_joint_config.joint_state = self.joint_statesmsg
            else:
                # Call compute IK
                ik_request_message_start = IkGoalRqstMsg()
                ik_request_message_start.position = request.start_pos.position
                ik_request_message_start.orientation = request.start_pos.orientation
                start_in_joint_config = RobotState()
                start_in_joint_config = await self.ik_callback(
                    ik_request_message_start, start_in_joint_config
                )
        if not request.is_xyzrpy:
            self.get_logger().info("CHANGING TO SEND HOME STATE")
            self.state = State.HOME_MSG
        elif not request.goal_pos.position and self.state == State.IDLE:
            # print('CHANGED TO PLAN ROT')
            self.get_logger().info("CHANGING TO PLAN ROT MSG STATE")
            self.state = State.ROT_MSG
        elif not request.goal_pos.orientation and self.state == State.IDLE:
            # print('CHANGED TO CART TRANS')
            self.get_logger().info("CHANGING TO CART MSG STATE")
            self.state = State.CART_MSG
        if self.state == State.ROT_MSG:
            # print('MAKING ROT MSG')
            self.get_logger().info(" CURRENT STATE {self.state}")
            self.get_logger().info("MAKING ROT MSG")
            ik_request_message_goal = IkGoalRqstMsg()
            ik_request_message_goal.position = request.goal_pos.position
            ik_request_message_goal.orientation = request.goal_pos.orientation

            goal_in_joint_config = RobotState()
            goal_in_joint_config = await self.ik_callback(
                ik_request_message_goal, goal_in_joint_config
            )

            plan_msg = self.get_motion_request(
                start_in_joint_config, goal_in_joint_config, request.execute_now
            )

            self.future_response = await self._plan_client.send_goal_async(plan_msg)
            self.plan_response = await self.future_response.get_result_async()

            self.get_logger().info("CHANGING TO PLAN EXEC STATE")
            self.state = State.PLAN_EXEC
            self.get_logger().info(f"CURRENT STATE {self.state}")
            # print(self.state)
            if request.execute_now==True:
                self.get_logger().info(f" 'EXECUTE NOW' TRUE-- EXECUTING PLAN REQUEST")
                self.get_logger().info(f"CHANGING TO IDLE STATE")
                self.state=State.IDLE

        

        if self.state==State.CART_MSG:
            self.get_logger().info(f" CURRENT STATE {self.state}")
            self.get_logger().info("MAKING CART MSG")
            if not request.goal_pos.orientation:

                request.goal_pos.orientation = [
                    self.ee_base.transform.rotation.x,
                    self.ee_base.transform.rotation.y,
                    self.ee_base.transform.rotation.z,
                    self.ee_base.transform.rotation.w,
                ]
            if not request.goal_pos.position:
                request.goal_pos.position= [self.ee_base.transform.translation.x,
                                self.ee_base.transform.translation.y,
                                self.ee_base.transform.translation.z]


            # print(f"\n GOAL POSE {request.goal_pos}")

            plan_msg = self.create_cart_msg(
                start_in_joint_config,
                request.goal_pos,
            )

            # print(f"\n CART REQ MSG {plan_msg}")

            
            # print(f"\n CART PLAN RESP {plan_msg.cart_request}")
            self.cart_response = await self.cart_client.call_async(GetCartesianPath.Request(header=plan_msg.cart_request.header, 
                                                                                            start_state=plan_msg.cart_request.start_state,
                                                                                            group_name=plan_msg.cart_request.group_name,
                                                                                            max_step=plan_msg.cart_request.max_step,
                                                                                            jump_threshold=plan_msg.cart_request.jump_threshold,
                                                                                            prismatic_jump_threshold=plan_msg.cart_request.prismatic_jump_threshold,
                                                                                            revolute_jump_threshold=plan_msg.cart_request.revolute_jump_threshold,
                                                                                            waypoints=plan_msg.cart_request.waypoints,))
            # self.plan_response = await self.future_response.get_result_async()
            # print(f"\n CART PLAN RESP {self.cart_response}")

            if self.state==State.CART_MSG:

                self.get_logger().info(f"CHANGING TO CART EXEC STATE")
                # print('SENDING CART MSG')
                self.state=State.CART_EXEC


        if self.state==State.HOME_MSG:
            self.state=State.HOME_MSG
            self.get_logger().info(f"SENDING HOME")
            self.get_logger().info(f"CURRENT STATE {self.state}")

            # FOR SENDING HOME JOINT STATES
            request.start_pos.position = self.joint_statesmsg.position
            start_in_joint_config = RobotState()
            start_in_joint_config.joint_state = self.joint_statesmsg

            goal_in_joint_config = RobotState()
            goal_in_joint_config.joint_state.position= self.joint_statesmsg.position ## GOAL POS IS JUST JOINT STATES
            plan_msg = self.get_motion_request(
                start_in_joint_config, goal_in_joint_config, request.execute_now
            )


            self.future_response = await self._plan_client.send_goal_async(plan_msg)
            self.plan_response = await self.future_response.get_result_async()

            self.get_logger().info("CHANGING TO PLAN EXEC STATE")
            self.state = State.PLAN_EXEC
            # print('SENDING ROT MSG')
            self.get_logger().info(f"CURRENT STATE {self.state}")
            # print(self.state)
            if request.execute_now == True:
                self.get_logger().info(" 'EXECUTE NOW' TRUE-- EXECUTING PLAN REQUEST")
                self.get_logger().info("CHANGING TO IDLE STATE")
                self.state = State.IDLE
        return response

    def send_execute(self):
        """
        Make the execute message to be used by the execute callback.

        :return: Message to send to the callback function to execute the
        planned trajectory.
        :rtype: ExecuteTrajectory_Goal

        """
        execute_msg = ExecuteTrajectory.Goal()

        if self.state == State.CART_EXEC:

            self.get_logger().info(f" CURRENT STATE {self.state}")
            execute_msg.trajectory = self.cart_response.solution

            # self.state=State.IDLE

        if self.state==State.PLAN_EXEC:
            print('EXECUTING ROT MSG')
            print(self.state)
            execute_msg.trajectory = self.plan_response.result.planned_trajectory
            print("SWITCHED TO IDLE")
            self.state = State.IDLE
            print(self.state)
        return execute_msg

    async def execute_callback(self, request, response):
        """
        Set the robot arm along the trajectory path generated by the plan callback.

        The robot will move in RViz and the physical robot will follow this movement.

        :param request: Empty message to execute the planned trajectory.
        :param response: Returns an empty value.

        :rtype: std_srvs.srv.Empty.Response()

        """
        exec_msg = self.send_execute()

        if self.state == State.CART_EXEC:
            self.get_logger().info(" EXECUTING CART MESSAGE")
        if self.state == State.PLAN_EXEC:
            self.get_logger().info(" EXECUTING PLAN MESSAGE")
        self.future_response2 = await self._execute_client.send_goal_async(exec_msg)
        self.execute_response = await self.future_response2.get_result_async()
        self.get_logger().info(" FINISHED EXECUTING- CHANGING TO IDLE")
        self.state = State.IDLE
        response = Empty.Response()

        return response

    def timer_callback(self):
        """
        Set up the transform listener to get the end effector position information.
        """
        try:
            self.ee_base = self.tf_buffer.lookup_transform(
                "panda_link0", "panda_hand_tcp", rclpy.time.Time()
            )

            self.get_logger().info("Panda TF data Status:OK", once=True)
        except:
            self.get_logger().warn("Panda TF data Status:MISSING")


def main(args=None):
    rclpy.init(args=args)
    newp = MoveBot()
    rclpy.spin(newp)
