'''
UNTESTED!

This node is used to control the gripper. Right now you can test it by running the commands shown below
You do not need to build the moveBot package for this to work since for now it is just a python script that 
creates a new ROS node. 

# to grasp an object, adjust the WIDTH, FORCE, SPEED, EPSILON values below if desired, then run
python3 gripper.py grasp 

# to home the gripper run
python3 gripper.py home

# to close the gripper
python3 gripper.py close

# to open the gripper
python3 gripper.py open



Simply running the commands above should work, although there may be a few small 
bugs that I missed when writing this since I haven't tested all of it. open() and close() should work 
for sure though.


Alternatively you can call the actions directly from the command line:


## Grasp 
ros2 action send_goal /panda_gripper/grasp franka_msgs/action/Grasp "width: 0.0312
epsilon:
  inner: 0.005
  outer: 0.005
speed: 1.0
force: 50.0
"


## Home
ros2 action send_goal /panda_gripper/homing franka_msgs/action/Homing {}

'''

# Hardcoded values for grasp action for testing now
WIDTH = 0.03
FORCE = 50.0
SPEED = 1.0
EPSILON = (0.005,0.005)


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp
from franka_msgs.action import Homing
import sys

class Gripper(Node):
    '''
    Node which creates an action client to send goals
    to /panda_gripper/gripper_action to open and close the gripper.
    This is currently just used for testing and will later be implemented
    either in the movebot API or the node(s) used for the project itself
    '''

    def __init__(self):
        super().__init__('gripper_action_client')
        self._gripper_action_client = ActionClient(self, GripperCommand,'/panda_gripper/gripper_action')
        self._grasp_client = ActionClient(self, GripperCommand,'/panda_gripper/grasp')
        self._homing_client = ActionClient(self, Homing, '/panda_gripper/homing')
        
    def grasp(self,width,speed=1.0,force=10.0,epsilon=(0.005,0.005)):
        '''
        Grasps an object.
        Uses the /panda_gripper/Grasp action
        '''
        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.speed = speed
        goal_msg.force = force
        #  goal_msg.epsilon.inner = epsilon[0]
        #  goal_msg.epsilon.outer = epsilon[1]
        self._grasp_client.wait_for_server()
        return self._grasp_client.send_goal_async(goal_msg)
    

    def home(self):
        '''
        Homes the gripper
        Uses the /panda_gripper/homing action
        '''

        goal_msg = Homing.Goal()
        self._homing_client.wait_for_server()
        return self._homing_client.send_goal_async(goal_msg)
        

    def close(self):
        '''
        Closes the gripper. Position=0.01 is closed for some reason
        Uses the /panda_gripper/gripper_action action
        '''

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.01
        goal_msg.command.max_effort = 1.0
        self._gripper_action_client.wait_for_server()
        return self._gripper_action_client.send_goal_async(goal_msg)
   
    def open(self):
        '''
        Opens the gripper. Position=0.04 is open for some reason
        Uses the /panda_gripper/gripper_action action
        '''

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.04
        goal_msg.command.max_effort = 1.0
        self._gripper_action_client.wait_for_server()
        return self._gipper_action_client.send_goal_async(goal_msg)
       
# get command line argument from user
help_msg = "Missing input argument\nAvailable arguments:\nopen\nclose\ngrasp\nhome"
assert(len(sys.argv) == 2),help_msg
cmd = sys.argv[1].lower()


def main(args=None):
    try:
        rclpy.init(args=args)
        action_client = Gripper()

        # this is just useful for testing, not needed when later integrated with API
        if cmd == "close":
            print("Closing...")
            future = action_client.close()
        elif cmd == "open":
            print("Opening...")
            future = action_client.open()
        elif cmd == "home":
            print("Homing...")
            future = action_client.home()
        elif cmd == "grasp":
            print("Grasping...")
            future = action_client.grasp(width=WIDTH,speed=SPEED,force=FORCE,epsilon=EPSILON)
        else:
            raise ValueError(f"Unrecognized argument {cmd}")

        rclpy.spin_until_future_complete(action_client, future)

    except KeyboardInterrupt:
        print("\nExited by user")


if __name__ == '__main__':
    main()
