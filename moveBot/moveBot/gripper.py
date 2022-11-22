
'''
Node which creates action clients to control the 
panda gripper. The actions used are shown in the
table below


|  function  |         action name           |
---------------------------------------------
| homing     | panda_gripper/homing 
| grasping   | panda_gripper/grasp
| open/close | panda_gripper/gripper_action


At the bottom of this file there is a main() function commented out
that shows how you could use this node by itself however what we actually
want to do is integrate this code into the API node (probably)
'''

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp
from franka_msgs.action import Homing
import sys

class Gripper(Node):
    '''
    Node which creates action clients to send goals
    to control the panda gripper.
    '''

    def __init__(self):
        super().__init__('movebot_gripper_client')
        self._gripper_action_client = ActionClient(self, GripperCommand,'/panda_gripper/gripper_action')
        self._grasp_client = ActionClient(self, Grasp,'/panda_gripper/grasp')
        self._homing_client = ActionClient(self, Homing, '/panda_gripper/homing')
       

    def grasp(self,width,speed=1.0,force=10.0,epsilon=(0.005,0.005)):
        '''
        Grasps an object. It can fail if the width is not accurate

        :param width: width of the object you are grasping
        :param speed: speed the gripper will close
        :param force: force the gripper will grasp the object
        :param epsilon: inner and outer tolerance

        '''
        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.speed = speed
        goal_msg.force = force
        goal_msg.epsilon.inner = epsilon[0]
        goal_msg.epsilon.outer = epsilon[1]
        self._grasp_client.wait_for_server()
        return self._grasp_client.send_goal_async(goal_msg)
    

    def home(self):
        '''
        Homes the gripper by first closing the gripper, then opening all the way.
        
        :return: A future object from the ActionClient.send_goal_async() function

        '''

        goal_msg = Homing.Goal()
        self._homing_client.wait_for_server()
        return self._homing_client.send_goal_async(goal_msg)
        

    def close(self):
        '''
        Closes the gripper, position=0.01 is close for some reason

        :return: A future object from the ActionClient.send_goal_async() function

        '''

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.01
        goal_msg.command.max_effort = 1.0
        self._gripper_action_client.wait_for_server()
        return self._gripper_action_client.send_goal_async(goal_msg)
   
    def open(self):
        '''
        Opens the gripper, position=0.04 is open for some reason
        
        :return: A future object from the ActionClient.send_goal_async() function
       
        '''

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.04
        goal_msg.command.max_effort = 1.0
        self._gripper_action_client.wait_for_server()
        return self._gripper_action_client.send_goal_async(goal_msg)
      


#  def main(args=None):
    #  rclpy.init(args=args)
    #  action_client = Gripper()
    #  future = action_client.home() # homes the gripper
    #  rclpy.spin_until_future_complete(action_client, future)
#
#  if __name__ == '__main__':
    #  main()
