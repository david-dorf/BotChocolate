
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
from moveit_msgs.msg import PositionIKRequest, MotionPlanRequest, Constraints, JointConstraint, PositionConstraint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup

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
        self.call_ik    = self.create_service(Empty,"call_ik",self.ik_callback,callback_group=self.cbgroup)
        self.call_plan    = self.create_service(Empty,"call_plan",self.plan_callback,callback_group=self.cbgroup)
        self.call_execute   = self.create_service(Empty,"call_execute",self.execute_callback,callback_group=self.cbgroup)
        self.time=0


    def js_cb(self, jointstate):
        """ 
        Callback function of the goal pose subscriber
        Stores the GoalPose message recieved 
         """
        self.joint_statesmsg=jointstate 
        # self.get_logger().info(f'goal msg {self.joint_statesmsg}')

    def get_ik(self):
        ikmsg = PositionIKRequest()
        ikmsg.group_name = 'panda_manipulator'
        ikmsg.robot_state.joint_state = self.joint_statesmsg
        print(self.joint_statesmsg)

        ikmsg.pose_stamped.header.frame_id = 'panda_link0'
        ikmsg.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ikmsg.pose_stamped.pose.position.x = 0.5
        ikmsg.pose_stamped.pose.position.y = 0.5
        ikmsg.pose_stamped.pose.position.z = 0.5
        ikmsg.pose_stamped.pose.orientation.x = 0.0
        ikmsg.pose_stamped.pose.orientation.y = 0.0
        ikmsg.pose_stamped.pose.orientation.z = 0.0
        ikmsg.pose_stamped.pose.orientation.w = 0.0
        ikmsg.timeout.sec = 5
        
        return ikmsg
        # self.ik_future = self.ik_client.call_async(GetPositionIK.Request(ik_request = ikmsg))

        # self.response=GetPositionIK.Response()
        # self.get_logger().info(f'response{self.response}')

    async def ik_callback(self,request,response):
 
        msg=self.get_ik()
        
        self.ik_response=await self.ik_client.call_async(GetPositionIK.Request(ik_request=msg))
        # self.response=GetPositionIK.Response()
        self.get_logger().info(f'\nresponse\n{self.ik_response}')
        #self.get_motion_request(self.ik_response)
        response=Empty.Response()
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
            print("TEST\n",self.joint_statesmsg.name[i])
            joint_constraints = JointConstraint()
            joint_constraints.joint_name = self.joint_statesmsg.name[i]
            joint_constraints.position = self.ik_response.solution.joint_state.position[i]
            joint_constraints.tolerance_above = 0.0001
            joint_constraints.tolerance_below = 0.0001
            joint_constraints.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraints)
            #goal_constraints.joint_constraints[i] = joint_constraints
        print("GOAL CONSTRAINTS\n", goal_constraints)
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
        print("Plan request:\n")
        print(plan_request)
        return plan_request

    async def plan_callback(self,request,response):
        plan_msg=self.send_plan()
        self.future_response=await self._plan_client.send_goal_async(plan_msg)
        # self.response=GetPositionIK.Response()
        self.plan_response=await self.future_response.get_result_async()
        self.get_logger().info(f'\nPlan rRsponse:\n{self.plan_response}')
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
        self.get_logger().info(f'\nresponse\n{self.execute_response}')

        response=Empty.Response()
        
        return response 







def main(args=None):
    rclpy.init(args=args)

    newp = Testing()
    rclpy.spin(newp)





