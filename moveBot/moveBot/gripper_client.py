import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
import sys

class GripperActionClient(Node):
    '''
    Node which creates an action client to send goals
    to /panda_gripper/gripper_action to open and close the gripper.
    This is currently just used for testing and will later be implemented
    either in the movebot API or the node(s) used for the project itself
    
    Open the gripper by running this command:
    python3 gripper_client open 

    Close the gripper by running this command:
    python3 gripper_client close

    '''
    def __init__(self):
        super().__init__('gripper_action_client')
        self._action_client = ActionClient(self, GripperCommand,'/panda_gripper/gripper_action')

    def close(self):
        '''
        Sends goal to gripper_action server to close the gripper
        with the desired effort. For some reason 0.01 seems to be
        the position corresponding to "close"
        '''
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.01
        goal_msg.command.max_effort = 60.0
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
   
    def open(self):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.04
        goal_msg.command.max_effort = 0.01
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
       
# get command line argument from user
assert(len(sys.argv) == 2),'Please specify either "close" or "open"'
cmd = sys.argv[1].lower()

def main(args=None):
    try:
        rclpy.init(args=args)
        action_client = GripperActionClient()

        # this is just useful for testing, we will want to logic
        # of our code to later specify open or close, not the user
        if cmd == "close":
            print("Closing...")
            future = action_client.close()
        elif cmd == "open":
            print("Opening...")
            future = action_client.open()
        else:
            raise ValueError(f"Unrecognized argument {cmd}")

        rclpy.spin_until_future_complete(action_client, future)

    except KeyboardInterrupt:
        print("\nExited by user")


if __name__ == '__main__':
    main()
