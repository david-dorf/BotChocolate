# MoveBot Interfaces

The movebot_interfaces package creates additional service and message types used by the MoveBot API
for interacting with the Franka Emika Panda robot arm using ROS2 MoveIt!.

## Messages 

**IkGoalRqstMsg.msg** (movebot_interfaces.msg): Used for creating lists of position and orientation 

**CartIKRequest** (movebot_interfaces.msg): Used to give parameters needed to 
generate a cartesian trajectory.

**SimpleCartPath** (movebot_interfaces.msg): Defines a new message of type CartIKRequest to simplify the API.

## Services 

**AddBox.srv** (movebot_interfaces.srv) - Used to give the dimensions of the box object added to the planning scene

**GetPlanRqst.srv** (movebot_interfaces.srv) - Used to give the start and end poses for path planning, and "execute" plan option

**IkGoalRqst.srv** (movebot_interfaces.srv) - Used to send the given position and orientation store the returned joint states



