
# Interface Package Description
The movebot_interfaces package contains all the services and messages necessary for interacting with the Franka Emika robot arm.
There are three services and one message:

1. IkGoalRqstMsg.msg, Type: movebot_interfaces.msg - Used for creating lists of position and orientation 

2. AddBox.srv, Type: movebot_interfaces.srv - Used to give the dimensions of the box object added to the planning scene

3. GetPlanRqst.srv, Type: movebot_interfaces.srv - Used to give the start and end poses for path planning, and "execute" plan option

4. IkGoalRqst.srv, Type: movebot_interfaces.srv - Used to send the given position and orientation store the returned joint states



