## Movebot
# ROS2 MoveIt2 API For Trajectory Planning 

Group Members: James Oubre, Allan Garcia, Shantao Cao, Nick Marks, David Dorf

## Package Description
The Movebot package includes a simple_move node that offers a sequence of services and actions that allow the 
Franka Emika robotic arm to plan and execute a trajectory through space while avoiding stationary obstacles. 
Note that the API currently  has joint values specific to the Franka Emilka arm. These can be removed when trying 
to use the API for any other robot model. 

To run the node use `ros2 run movebot simple_move`. 

The package also includes a launch file that can be included into other package launch files.
To launch the node via the launch command use `ros2 launch movebot simple_move.launch.py`. 

## Operation
Package calls services in order create a desired trajectory to a goal position. More information on the messages and services used can be found [here.](./movebot_interfaces) 


### Create and Execute a Trajectory
The main service used to input a trajectroy is given below:
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

If the goal orientation field is left empty, the inputted goal position will be planned 
a Cartesian trajectory using the GetCartesianPath service.If the goal position field is left 
empty, the inputted goal orientation will be planned through the MoveGroup action.

Lastly, there are two flags to alter the operation of system. By setting `is_xyzrpy` the user may
specify that the start and end coordinates being input are in Cartesian coordinates and not a
joint state vector. By setting `execute_now` to true, the robot will immediately execute the planned
trajectory. Otherwise, a separate service must be called to execute the trajectory.

By setting `execute_now` to false, the user can execute a trajectory by calling the `/call_execute` service
one of two ways: <br />
    1. In the terminal: `ros2 service call /call_execute std_srvs/srv/Empty`
    2. By creating a client for the service in another node


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

# Demonstration

[robot_move.webm](https://user-images.githubusercontent.com/46512429/201226143-f7d638e7-5f5c-4de6-9c8c-1f0e36112dcc.webm)
