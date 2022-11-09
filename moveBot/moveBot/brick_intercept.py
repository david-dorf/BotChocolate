import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
import moveit_msgs
import std_msgs
import builtin_interfaces.msg
import geometry_msgs
from geometry_msgs.msg import Pose
import octomap_msgs
import sensor_msgs
import trajectory_msgs
from moveit_msgs.msg import PositionIKRequest, MotionPlanRequest, Constraints, JointConstraint, PositionConstraint,CollisionObject,PlanningScene, PlanningSceneComponents
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene, GetPlanningScene
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from shape_msgs.msg import SolidPrimitive

class Brick(Node):
    def __init__(self):
        super().__init__("Brick")
        self.cbgroup = ReentrantCallbackGroup()
        # self._plan_client = ActionClient(
        #     self, 
        #     MoveGroup,
        #     "move_action",)

        # self._execute_client = ActionClient(
        #     self, 
        #     ExecuteTrajectory,
        #     "execute_trajectory")


        #self.jointpub  = self.create_subscription(JointState, "/joint_states",self.js_cb, 10)
        #self.ik_client= self.create_client(GetPositionIK, "/compute_ik",callback_group=self.cbgroup)
        #self.call_ik    = self.create_service(Empty,"call_ik",self.ik_callback,callback_group=self.cbgroup)
        #self.call_plan    = self.create_service(Empty,"call_plan",self.plan_callback,callback_group=self.cbgroup)
        #self.call_execute   = self.create_service(Empty,"call_execute",self.execute_callback,callback_group=self.cbgroup)
        #self.time=0

        ##self.brick_ser = self.create_service(ApplyPlanningScene,"apply_planning_scene",self.brick_callback, callback_group=self.cbgroup)
        self.box_publisher = self.create_publisher(PlanningScene,"planning_scene",10)
        self.call_box = self.create_service(Empty,"call_box",self.box_callback,callback_group=self.cbgroup)
        self.scene_client = self.create_client(GetPlanningScene,"get_planning_scene",callback_group=self.cbgroup)

    async def box_callback(self,request,response):
        #Scene = PlanningScene()
        component = PlanningSceneComponents()
        Scene_raw = await self.scene_client.call_async(GetPlanningScene.Request(components = component))
        self.get_logger().info(f'\nresponse\n{Scene_raw}')
        Scene = Scene_raw.scene
        box = CollisionObject()
        box.header.stamp = self.get_clock().now().to_msg()
        box.header.frame_id = "panda_link0"
        box.pose.position.x = 1.0
        box.pose.position.y = 2.0
        box.pose.position.z = 3.0
        box.pose.orientation.x = 0.0
        box.pose.orientation.y = 0.0
        box.pose.orientation.z = 0.0
        box.pose.orientation.w = 1.0
        box.id = "box0"
        SP = SolidPrimitive()
        SP.type = 1
        SP.dimensions = [0.2,0.2,0.2]
        box.primitives = [SP]

        SPPose = Pose()
        box.primitive_poses = [SPPose]

        Scene.world.collision_objects.append(box)

        self.get_logger().info(f'\nresponse\n{Scene}')

        self.box_publisher.publish(Scene)

        return Empty.Response()


    # async def brick_callback(self,request,response):
    #     self.get_logger().info(f'\nbrick_request:\n{request}')

    #     return ApplyPlanningScene.Response()


            





def main(args=None):
    rclpy.init(args=args)

    newp = Brick()
    rclpy.spin(newp)
    #rclpy.shutdown()