
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
import moveit_msgs
import std_msgs
import builtin_interfaces.msg
import geometry_msgs
import octomap_msgs
import sensor_msgs
import trajectory_msgs
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PositionIKRequest, MotionPlanRequest, Constraints, JointConstraint, RobotState, CollisionObject,PlanningScene, PlanningSceneComponents

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK,GetPlanningScene
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from shape_msgs.msg import SolidPrimitive
import math
import numpy as np
from movebot_interfaces.srv import IkGoalRqst, GetPlanRqst, AddBox
from movebot_interfaces.msg import IkGoalRqstMsg

def quaternion_from_euler(ai, aj, ak):
    """
    Take in Euler angles and converts them to quaternions. Function taken from link above.

    Args:
    ----
        ai (float): Roll angle.
        aj (float): Pitch angle.
        ak (float): Yaw angle.

    Return:
    ------
        float array: Array of quaternion angles.

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

    q = np.empty((4, ))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q

class Testing(Node):
    def __init__(self):
        super().__init__("fake_movegroup")
        self.cbgroup = ReentrantCallbackGroup()
        self._plan_client = ActionClient(
            self, 
            MoveGroup,
            "move_action",)

        self._execute_client = ActionClient(
            self, 
            ExecuteTrajectory,
            "execute_trajectory")


        self.jointpub  = self.create_subscription(JointState, "/joint_states",self.js_cb, 10)
        self.ik_client= self.create_client(GetPositionIK, "/compute_ik",callback_group=self.cbgroup)
        self.call_ik    = self.create_service(IkGoalRqst,"call_ik",self.ik_callback,callback_group=self.cbgroup)
        self.call_plan    = self.create_service(GetPlanRqst,"call_plan",self.plan_callback,callback_group=self.cbgroup)
        self.call_execute   = self.create_service(Empty,"call_execute",self.execute_callback,callback_group=self.cbgroup)
        self.timer = self.create_timer(1/100, self.timer_callback)
        self.box_publisher = self.create_publisher(PlanningScene,"planning_scene",10)
        self.call_box = self.create_service(Empty,"call_box",self.box_callback,callback_group=self.cbgroup)
        self.clear_all_box = self.create_service(Empty,"clear_all_box",self.clear_callback,callback_group=self.cbgroup)
        self.clear_current_box = self.create_service(Empty,"clear_current_box",self.remove_callback,callback_group=self.cbgroup)
        self.scene_client = self.create_client(GetPlanningScene,"get_planning_scene",callback_group=self.cbgroup)
        self.update_box = self.create_service(AddBox,"add_box",self.update_box_callback)
        self.box_x = 0.2
        self.box_y = 0.2
        self.box_z = 0.2
        self.box_l = 0.2
        self.box_w = 0.2
        self.box_h = 0.2
        self.box_name = "box_0"


        # self.broadcaster = TransformBroadcaster(self)
        # self.ee_base = TransformStamped()
        # self.ee_base.header.frame_id = "panda_link0"
        # self.ee_base.child_frame_id = "panda_hand"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.time=0

    def update_box_callback(self,request,response):

        self.box_x = request.x
        self.box_y = request.y
        self.box_z = request.z
        self.box_l = request.l
        self.box_w = request.w
        self.box_h = request.h
        self.box_name = str(request.name)
        return response

    async def box_callback(self,request,response):
        #Scene = PlanningScene()
        component = PlanningSceneComponents()
        Scene_raw = await self.scene_client.call_async(GetPlanningScene.Request(components = component))
        self.get_logger().info(f'\nresponse\n{Scene_raw}')
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
        SP.dimensions = [self.box_l,self.box_w,self.box_h]
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


        self.get_logger().info(f'\nresponse\n{Scene}')

        self.box_publisher.publish(Scene)

        return Empty.Response()

    async def clear_callback(self,request,response):
        #Scene = PlanningScene()
        component = PlanningSceneComponents()
        Scene_raw = await self.scene_client.call_async(GetPlanningScene.Request(components = component))
        self.get_logger().info(f'\nresponse\n{Scene_raw}')
        Scene = Scene_raw.scene

        Scene.world.collision_objects = []

        self.get_logger().info(f'\nresponse\n{Scene}')

        self.box_publisher.publish(Scene)

        return Empty.Response()

    async def remove_callback(self,request,response):
        #Scene = PlanningScene()
        component = PlanningSceneComponents()
        Scene_raw = await self.scene_client.call_async(GetPlanningScene.Request(components = component))
        self.get_logger().info(f'\nresponse\n{Scene_raw}')
        Scene = Scene_raw.scene

        for i in Scene.world.collision_objects:
            if i.id == self.box_name:
                Scene.world.collision_objects.remove(i)
                break

        self.get_logger().info(f'\nresponse\n{Scene}')

        self.box_publisher.publish(Scene)

        return Empty.Response()

    def js_cb(self, jointstate):
        """ 5
        Callback function of the goal pose subscriber
        Stores the GoalPose message recieved 
         """
        self.joint_statesmsg=jointstate 
        #self.get_logger().info(f'goal msg {self.joint_statesmsg}')

    def get_ik_rqst_msg(self, pose_vec):
        # self.get_logger().info(f'\nPose_VEC IN GET_IK_RQST\n{pose_vec}')
        # if pose_vec[0]==0.0 and pose_vec[1]==0.0 and pose_vec[2]==0.0:
        #     # self.get_logger().info(f'\nPoses\n{pose_vec}')
        #     print("ahh")
        #     pose_vec[0],pose_vec[1],pose_vec[2]=self.ee_base.transform.translation.x,self.ee_base.transform.translation.y,self.ee_base.transform.translation.z

        ikmsg = PositionIKRequest()
        ikmsg.group_name = 'panda_manipulator'
        ikmsg.robot_state.joint_state = self.joint_statesmsg
        # print("\nANGLES\n", pose_vec[3], pose_vec[4], pose_vec[5])

        # print(self.joint_statesmsg)

        ikmsg.pose_stamped.header.frame_id = 'panda_link0'
        ikmsg.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ikmsg.pose_stamped.pose.position.x = pose_vec[0]
        ikmsg.pose_stamped.pose.position.y = pose_vec[1]
        ikmsg.pose_stamped.pose.position.z = pose_vec[2]
        quats = quaternion_from_euler(pose_vec[3],pose_vec[4],pose_vec[5])
        ikmsg.pose_stamped.pose.orientation.x = quats[0]
        ikmsg.pose_stamped.pose.orientation.y = quats[1]
        ikmsg.pose_stamped.pose.orientation.z = quats[2]
        ikmsg.pose_stamped.pose.orientation.w = quats[3]
        ikmsg.timeout.sec = 5

        #self.get_logger().info(f'\nIk msg Request\n{ikmsg}')
        return ikmsg

    async def ik_callback(self,request,response):
        #self.get_logger().info(f'\nRequest for ik callback t\n{request}')
        if not request.position and not request.orientation:
            current_position = [self.ee_base.transform.translation.x, \
                                self.ee_base.transform.translation.y, \
                                self.ee_base.transform.translation.z]
            current_orientation = [0.0, 0.0, 0.0]
            pose_vec = np.hstack([current_position, current_orientation])
            print("POSE when everything is fucked\n", pose_vec)

        elif not request.position:
            current_position = [self.ee_base.transform.translation.x, \
                                self.ee_base.transform.translation.y, \
                                self.ee_base.transform.translation.z]

            pose_vec = np.hstack([current_position, request.orientation])
            # print("POSE VEC AFTER POSITION\n", pose_vec)
        elif not request.orientation:
            current_orientation = [0.0, 0.0, 0.0] # TODO make so that orientation doesn't change
            pose_vec = np.hstack([request.position, current_orientation])
            print("POSE VEC AFTER ORIENT\n", pose_vec)
        else: 
            pose_vec = np.hstack([request.position, request.orientation])
            print("POSE VEC NO CHANGE\n", pose_vec)

        msg=self.get_ik_rqst_msg(pose_vec)
        print("POSE VEC AFTER EVERYTHING\n", pose_vec)

        self.ik_response = await self.ik_client.call_async(GetPositionIK.Request(ik_request=msg))
        # self.response=GetPositionIK.Response()
        # self.get_logger().info(f'\nIk response\n{self.ik_response}')
        # self.get_logger().info(f'\nIk ik callback response\n{response}')
        response.joint_state = self.ik_response.solution.joint_state

        return response
    
    def get_motion_request(self, start, goal): # TODO take in start and end pose
        motion_req = MotionPlanRequest()
        motion_req.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        motion_req.workspace_parameters.min_corner.x = -1.0
        motion_req.workspace_parameters.min_corner.y = -1.0
        motion_req.workspace_parameters.min_corner.z = -1.0
        motion_req.workspace_parameters.max_corner.x = 1.0
        motion_req.workspace_parameters.max_corner.y = 1.0
        motion_req.workspace_parameters.max_corner.z = 1.0
        motion_req.workspace_parameters.header.frame_id = 'panda_link0'
        motion_req.start_state.joint_state = start.joint_state # TODO pass start stated that is compute ik'd
        
        goal_constraints = Constraints()
        print("GOALTEST\n", goal)
        #joint_constraints = JointConstraint()
        for i in range(len(self.joint_statesmsg.name)):
            # print("TEST\n",self.joint_statesmsg.name[i])
            joint_constraints = JointConstraint()
            joint_constraints.joint_name = self.joint_statesmsg.name[i]
            joint_constraints.position = goal.joint_state.position[i] # TODO instead of self.ik_response itll just joint goal position
            joint_constraints.tolerance_above = 0.0001
            joint_constraints.tolerance_below = 0.0001
            joint_constraints.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraints)
            #goal_constraints.joint_constraints[i] = joint_constraints
        # print("GOAL CONSTRAINTS\n", goal_constraints)
        motion_req.goal_constraints = [goal_constraints]
        motion_req.pipeline_id = 'move_group'
        motion_req.group_name = 'panda_manipulator'
        motion_req.num_planning_attempts = 10
        motion_req.allowed_planning_time = 5.0
        motion_req.max_velocity_scaling_factor = 0.1
        motion_req.max_acceleration_scaling_factor = 0.1
        motion_req.max_cartesian_speed = 0.0
        plan_request=MoveGroup.Goal()
        plan_request.request = motion_req
        plan_request.planning_options.plan_only = True
        return plan_request

    async def plan_callback(self,request,response):
        # TODO did you suply start pos
        # if not start_pos = current pos
        #If so is it xzy or joints
        # If xyz do compute_Ik
        #If joints start_pose = joints (be careful of msg types)
        #self.get_logger().info(f'\nIk ik callback response\n{response}')

        if request.is_xyzrpy: # If start pos was given as X,Y,Z, R, P, Y
            if not request.start_pos: # IF there is no given start position, use current joint config as start
                # print("NO GIVEN START POSE, USING CURRENT POSE AS START")
                request.start_pos=self.joint_statesmsg.position
                # TODO if goal is given as joint states, put as correct msg type to pass to motion request
                # else:
                
            # Call compute IK
            ik_request_message_start = IkGoalRqstMsg()
            # ik_request_message_start.position = [request.start_pos.position[0], request.start_pos.position[1], request.start_pos.position[2]]
            # ik_request_message_start.orientation = [request.start_pos.orientation[0], request.start_pos.orientation[1], request.start_pos.orientation[2]]
            # self.get_logger().info(f'\ngoal orienatation \n{request.goal_pos.orientation}')

            ik_request_message_start.position = request.start_pos.position
            ik_request_message_start.orientation = request.start_pos.orientation
            start_in_joint_config = RobotState()
            start_in_joint_config = await self.ik_callback(ik_request_message_start, start_in_joint_config)
            
            ik_request_message_goal = IkGoalRqstMsg()
            ik_request_message_goal.position = request.goal_pos.position
            ik_request_message_goal.orientation = request.goal_pos.orientation
            goal_in_joint_config = RobotState()
            goal_in_joint_config = await self.ik_callback(ik_request_message_goal, goal_in_joint_config)
        
        plan_msg=self.get_motion_request(start_in_joint_config, goal_in_joint_config) # TODO we want to send the start pos we get from the service
        # self.get_logger().info(f'\n SACKK \n')
        self.future_response=await self._plan_client.send_goal_async(plan_msg)
        # self.response=GetPositionIK.Response()
        self.plan_response=await self.future_response.get_result_async()
        # self.get_logger().info(f'\nPlan rRsponse:\n{self.plan_response}')
        # response=Empty.Response()
        # self.get_logger().info(f'\nIk ik callback response\n{response}')
        return response 

    def send_execute(self):
        execute_msg=ExecuteTrajectory.Goal()
        execute_msg.trajectory = self.plan_response.result.planned_trajectory

        return execute_msg

    async def execute_callback(self,request,response):
 
        exec_msg=self.send_execute()
        self.future_response=await self._execute_client.send_goal_async(exec_msg)
        self.execute_response=await self.future_response.get_result_async()
        # self.get_logger().info(f'\nresponse\n{self.execute_response}')

        response=Empty.Response()
        
        return response 

    def timer_callback(self):
        time = self.get_clock().now().to_msg()
        # self.ee_base.header.stamp = time
        # self.broadcaster.sendTransform(self.ee_base)

        try:
            self.ee_base = self.tf_buffer.lookup_transform('panda_link0','panda_hand',rclpy.time.Time())
            # self.get_logger().info(f'\n E.E X \n{self.ee_base.transform.translation.x}')
            # self.get_logger().info(f'\n E.E Y \n{self.ee_base.transform.translation.y}')
            # self.get_logger().info(f'\n E.E Z \n{self.ee_base.transform.translation.z}')
        except:
            pass


        # self.get_logger().info(f'\n E.E X \n{self.ee_base.transform.translation.x}')
        # self.get_logger().info(f'\n E.E Y \n{self.ee_base.transform.translation.y}')
        # self.get_logger().info(f'\n E.E Z \n{self.ee_base.transform.translation.z}')





def main(args=None):
    rclpy.init(args=args)

    newp = Testing()
    rclpy.spin(newp)





