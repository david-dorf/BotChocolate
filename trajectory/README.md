# Trajectory
A movement sequence program for the Franka Emika Panda robot arm using ROS2 MoveIt2 API

## Package Description
The trajectory package contains the trajectory node, which runs services that interact with the movebot API to command the Franka Emika robot's motion. The trajectory node allows the robot to move and manipulate objects in its surroundings to make hot chocolate (or other powdered beverages).

## Services
The trajectory node takes in position information as inputs from the computer vision algorithms in the bot_vis package. Additionally, a planning scene is created with collision box objects to prevent the robot from colliding with the tables in the scenario. Clients in the program use the services: `add_box`, `call_box`,`clear_all_box` from the BoxCaller class of the movebot API. For motion requests, clients use `call_plan`, `call_cart`, and `call_execute` from the joint state component of the movebot API.

## Operation
To run the node independent of the system launchfile, use the command `ros2 run moveBot trajectory`. Once running, the command `ros2 service call /make_hot_chocolate std_srvs/srv/Empty` executes the plan through the sequence of the calculated waypoints.

## Motion
The robot utilizes both rotational motion planning and cartesian trajectories to achieve motions like pouring. Returning to predefined waypoints and the home position prevent the robot from reaching joint limits while it moves through a complex trajectory that takes it close to the edge of the robot's workspace.

