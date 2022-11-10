# ME495 Embedded Systems Homework 3 Part 2
Group: Bot Chocolate

Group Members: James Oubre, Allan Garcia, Shantao Cao, Nick Marks, David Dorf

## Package Description
The moveBot package offers a sequence of three services that allow the Franka Emika robot arm to
plan and execute a trajectory through space while avoiding stationary obstacles. RViz must first be
initialized for the Franka robot with the command `ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true` in the built and sourced nuws directory.

## Operation
Package calls services in the order create a trajectory to a goal position. More information on the messages and services used can be found [here.](./movebot_interfaces/README.md) 

### Call Launch File
`ros2 launch moveBot simple_move.launch.py`

### Create and Execute a Trajectory
The template to create trajectroy plans is given below:

`
    ros2 service call /call_plan movebot_interfaces/srv/GetPlanRqst 
    "start_pos:
    position: []
    orientation: []
    goal_pos:
    position: []
    orientation: []
    is_xyzrpy: true
    execute_now: true 
    "
`
The `start_pos` and `goal_pos` fields are where different trajectory start and end points can be 
specified. 

To use the current position of the end effector as a start position, the `start_pos` field can
be passed as empty.

Each pose field also has a position and orientation subfield. Here an end effector configuration
position may be specified in Cartesian coordinates and the end effector angle may be specified
as roll, pitch, and yaw in radians. Furthermore, to specify solely a position or solely an 
orientation, the opposite subfield may be entered as empty in the service call.

Lastly, there are two flags to alter the operation of system. By setting `is_xyzrpy` the user may
specify that the start and end coordinates being input are in Cartesian coordinates and not a
joint state vector. By setting `execute_now` to true, the robot will immediately execute the planned
trajectory. Otherwise, a separate service must be called to execute the trajectory.

By setting `execute_now` to false, the user can then execute a trajectroy by calling the following
service:
`ros2 service call /call_execute std_srvs/srv/Empty`

### Adding Obstacles to the Planning Scene
It is possible to add boxes of different sizes to the plannning scene for the robot to navigate 
around.

The following command will add a box to the planning scene:
`
        ros2 service call /add_box movebot_interfaces/srv/AddBox "name: '<obstacle_name>'
        x: 0.2
        y: 0.2
        z: 0.2
        l: 0.2
        w: 0.2
        h: 0.2
`

'ros2 service call /call_box std_srvs/srv/Empty'

To delete the last added box the following command may be used:
`ros2 service call /clear_current_box std_srvs/srv/Empty `

To clear all obstacle from the planning scene:
`ros2 service call /clear_all_box std_srvs/srv/Empty`