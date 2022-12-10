# BotChocolate - The Hot Chocolate Making Robot
Collaborators: James Oubre, Allan Garcia, Shantao Cao, Nick Marks, David Dorf

The purpose of this project was to create hot chocolate using the Franka Emika 7 DOF robot arm. To 
perceive the environment, the system utilizes an Intel D435i camera AprilTags. Upon initial setup, a
calibration process must be completed. After this, the process consisted of using AprilTags to 
locate a scoop of cocoa, a mug, a spoon, and a kettle relative to the robot. Next, using our custom 
MoveIt API for Python, movebot, the robot is able to perform path planning between all of these
objects. It turns on the kettle, dumps the cocoa powder into the mug, pours the hot water over the 
power, and stirs the mixture it with the spoon.

## System Architecture
BotChocolate consists of 4 packages:

`movebot` is a Python API for MoveIt in ROS2 used for path planning of the robot. It uses the `simple_move`
to interface with other nodes.

`movebot_interfaces` consists of the custom srv and msgs to interact with the movebot package.

`trajectory` package has the actual hot chocolate plan and uses the movebot package to create paths
between different waypoints necessary to make hot chocolate. It also contains the `trajectory` node
which sends desired waypoints to `simple_move` to move the robot.

`bot_vis` is the computer vision package for BotChocolate that uses AprilTag ROS2 packages to calibrate
the system and broadcast transformations of the hot chocolate components to the robot through the `april_tf`
node.

# Demonstration (Note: 1.5x Speed)

[botchoc_15speed.webm](https://user-images.githubusercontent.com/46512429/206770769-fba8fc8e-2711-4839-bb2b-fb6178e7839a.webm)

## Setup and External Packages
Before being able to run BotChocolate, you must ensure that you have all the necessary Franka 
packages installed (https://nu-msr.github.io/me495_site/franka.html).
It is convenient to have a workspace that is updated to the latest version of numsr_patches (https://github.com/m-elwin/numsr_patches)

Necessary external packages outside of numsr_patches can be installed with the botchoc.repos file using
`vcs import < botchoc.repos`.

## Operation 
0. If running for the first time, calibration must be done. To calibrate the system, the end-effector 
AprilTag must be placed in the robot's gripper, aligned with the panda_link_tcp frame, and in the 
field of view of the camera. Then call `ros2 launch bot_vis launch_vision.py calibration:=true`. 
This step can now be ignored for subsequent runs.

1. Plug the robot's Ethernet cable and the camera into your computer.

2. ssh into robot and go `https://panda0.robot` in your browser.

3. In the browser window, unlock the robot and active FCI.

4. In a separate terminal run `ros2 launch trajectory botchocolate.launch.py real:=true`.

5. In another terminal run `ros2 service call /make_hot_chocolate std_srvs/srv/Empty`

6. Enjoy hot chocolate.

## Documentation

Documentation for the BotChocolate project contains documentation for both the `trajectory` and `bot_vis` packages. Separate documentation
for the MoveBot API can be found in `movebot/docs`. For now, documentation is only viewable by building locally. To build it 
you must first install the `m2r2` python package. Run `pip3 install m2r2` before trying to build the docs. You may 
also need to install sphinx with `apt install python3-sphinx`.

To build the BotChocolate documentation,navigate the to `docs/` directory and run `make html`
(note you must have `make` installed). There now should be a new directory `_build/html`
and in it you will find a file, `index.html` which you can open in your web browser of choice to view the documentation.

To view the MoveBot API documentation, navigate to `movebot/docs` and again run `make html` and the `index.html` 
file will be located at `movebot/docs/_build/html/index.html`

### Launch Files
`launch_vision.py` launches the `april_tf`, `apriltag_node`, and `realsense` nodes as well as rviz and the `calibration` node if specified.

`simple_move.launch.py` launches ...

`botchocolate.launch.py` launches ...

## The Group

![bot_choc-min](https://user-images.githubusercontent.com/46512429/206768445-4503edc2-2075-48b4-baf7-e6dc7bd3ca86.png)
