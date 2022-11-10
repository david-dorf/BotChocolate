# ME495 Embedded Systems Homework 3 Part 2
Group: Bot Chocolate

Group Members: James Oubre, Allan Garcia, Shantao Cao, Nick Marks, David Dorf

## Package Description
The moveBot package offers a sequence of three services that allow the Franka Emika robot arm to
plan and execute a trajectory through space while avoiding stationary obstacles. RViz must first be
initialized for the Franka robot with the command `ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true` in the built and sourced nuws directory.

## Services
Package calls services in the order: call_ik, call_plan, call_execute

call_ik - `ros2 service call /call_ik movebot_interfaces/srv/IkGoalRqst`
Optional arguments:
`x: float y: float z: float roll: float pitch: float yaw: float`

Description: Computes the inverse kinematics solution for a desired end effector position.

call_plan - `ros2 service call /call_plan std_srvs/srv/Empty`
Description: Generates a trajectory plan for the robot arm to follow to reach the goal position.
The planned path is shown in RViz after the service is called.

call_execute - `ros2 service call /call_execute std_srvs/srv/Empty`
Description: Executes the planned trajectory. The robot will move in RViz to the goal position and
send commands to the physical robot to mimic the simulated trajectory.