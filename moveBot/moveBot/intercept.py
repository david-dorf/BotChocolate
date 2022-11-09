
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
from moveit_msgs.msg import PositionIKRequest, MotionPlanRequest, Constraints, JointConstraint, RobotState
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import numpy as np
from movebot_interfaces.srv import IkGoalRqst

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
        self.call_plan    = self.create_service(Empty,"call_plan",self.plan_callback,callback_group=self.cbgroup)
        self.call_execute   = self.create_service(Empty,"call_execute",self.execute_callback,callback_group=self.cbgroup)
        self.timer = self.create_timer(1/100, self.timer_callback)


        # self.broadcaster = TransformBroadcaster(self)
        # self.ee_base = TransformStamped()
        # self.ee_base.header.frame_id = "panda_link0"
        # self.ee_base.child_frame_id = "panda_hand"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.time=0


    def js_cb(self, jointstate):
        """ 
        Callback function of the goal pose subscriber
        Stores the GoalPose message recieved 
         """
        self.joint_statesmsg=jointstate 
        # self.get_logger().info(f'goal msg {self.joint_statesmsg}')

    def get_ik(self, pose_vec):
        self.get_logger().info(f'\nPoses\n{pose_vec}')
        if pose_vec[0]==0.0 and pose_vec[1]==0.0 and pose_vec[2]==0.0:
            # self.get_logger().info(f'\nPoses\n{pose_vec}')
            print("ahh")
            pose_vec[0],pose_vec[1],pose_vec[2]=self.ee_base.transform.translation.x,self.ee_base.transform.translation.y,self.ee_base.transform.translation.z

        ikmsg = PositionIKRequest()
        ikmsg.group_name = 'panda_manipulator'
        ikmsg.robot_state.joint_state = self.joint_statesmsg
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

        self.get_logger().info(f'\nIk msg\n{ikmsg}')
        return ikmsg

    async def ik_callback(self,request,response):
        pose_vec = np.hstack([request.position, request.orientation])
        msg=self.get_ik(pose_vec)

        self.ik_response = await self.ik_client.call_async(GetPositionIK.Request(ik_request=msg))
        # self.response=GetPositionIK.Response()
        self.get_logger().info(f'\nIk response\n{self.ik_response}')
        response.joint_states = self.ik_response.solution

        return response
    
    def get_motion_request(self):
        motion_req = MotionPlanRequest()
        motion_req.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        motion_req.workspace_parameters.min_corner.x = -1.0
        motion_req.workspace_parameters.min_corner.y = -1.0
        motion_req.workspace_parameters.min_corner.z = -1.0
        motion_req.workspace_parameters.max_corner.x = 1.0
        motion_req.workspace_parameters.max_corner.y = 1.0
        motion_req.workspace_parameters.max_corner.z = 1.0
        motion_req.workspace_parameters.header.frame_id = 'panda_link0'
        motion_req.start_state.joint_state = self.joint_statesmsg
        goal_constraints = Constraints()
        
        #joint_constraints = JointConstraint()
        for i in range(len(self.joint_statesmsg.name)):
            # print("TEST\n",self.joint_statesmsg.name[i])
            joint_constraints = JointConstraint()
            joint_constraints.joint_name = self.joint_statesmsg.name[i]
            joint_constraints.position = self.ik_response.solution.joint_state.position[i]
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
        
        return motion_req
        
    
        #mtn_reg.goal_constraints.joint_constraints = ik_resp. 
        # joint_names = self.ik_resp.name
        # joint_positions = self.ik_resp.position
        
    def send_plan(self):
        plan_request=MoveGroup.Goal()
        plan_request.request = self.get_motion_request()
        plan_request.planning_options.plan_only = True
        # print("Plan request:\n")
        # print(plan_request)
        return plan_request

    async def plan_callback(self,request,response):
        plan_msg=self.send_plan()
        self.future_response=await self._plan_client.send_goal_async(plan_msg)
        # self.response=GetPositionIK.Response()
        self.plan_response=await self.future_response.get_result_async()
        # self.get_logger().info(f'\nPlan rRsponse:\n{self.plan_response}')
        response=Empty.Response()
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
            self.get_logger().info(f'\n E.E X \n{self.ee_base.transform.translation.x}')
            self.get_logger().info(f'\n E.E Y \n{self.ee_base.transform.translation.y}')
            self.get_logger().info(f'\n E.E Z \n{self.ee_base.transform.translation.z}')
        except:
            pass


        # self.get_logger().info(f'\n E.E X \n{self.ee_base.transform.translation.x}')
        # self.get_logger().info(f'\n E.E Y \n{self.ee_base.transform.translation.y}')
        # self.get_logger().info(f'\n E.E Z \n{self.ee_base.transform.translation.z}')





def main(args=None):
    rclpy.init(args=args)

    newp = Testing()
    rclpy.spin(newp)





