# BotChocolate - The Hot Chocolate Making Robot
Collaborators: James Oubre, Allan Garcia, Shantao Cao, Nick Marks, David Dorf


The purpose of this project was to create hot chocolate using the Franka Emika 7 DOF robot arm. To 
percieve the environment, the system utilizes an Intel D435i camera AprilTags. Upon initial setup, a
calibration process must be completed. After this, the process consisted of using AprilTags to 
locate a scoop of cocoa, a mug, a spoon, and a kettle relative to the robot. Next, using our custom 
MoveIt API for python, movebot, the robot is able to perform path planning between all of these 
objects. It turns on the kettle, dumps the cocoa powder into the mug, pours the hot water over the 
power, and stirs the mixture it with the spoon.

## System Architecture
BotChocolate consists of 4 packages:

`movebot` is a python API for MoveIt in ROS2 used for path planning of the robot

`movebot_interfaces` consists of the custom srv and msgs to interact with the movebot package

`trajectory` package has the actual hot chocoalate plan and uses the movebot package to create paths
between different waypoints necessary to make hot chocolate

`bot_vis` is the computer vision package for BotChocolate that uses AprilTag ROS2 packages to calibrate
the system and broadcast tranformations of the hot chocolate components to the robot

# Demonstration (Note: 1.5x Speed)

[botchoc_15speed.webm](https://user-images.githubusercontent.com/46512429/206770769-fba8fc8e-2711-4839-bb2b-fb6178e7839a.webm)

## Setup and External Packages
Before being able to run BotChocolate, you must ensure that you have all the necessary Franka 
packages installed (https://nu-msr.github.io/me495_site/franka.html).
It is convenient to have a workspace that is updated to the latest version of numsr_patches (https://github.com/m-elwin/numsr_patches)

Necessary external packages outside of numsr_patches can be installed with the botchoc.repos file using
`vcs import < botchoc.repos`.

## Operation 
to run from scratch a calibration must first be done
Cailbrate the robot
To calibrate the system, the end-effector AprilTag must be placed in the robot's gripper, aligned with the panda_link_tcp frame, and in the field of view of the camera. Next, just call the launch launch file with the calibration flag.
kill the calibration terminal
this step can now be ignored for subsequent runs

ssh into robot and go thorugh franka setup

call launch file
call service
make botchocolate

## Documentation

Documentation for the BotChocolate project contains documentation for both the `trajectory` and `bot_vis` packages. Separate documentation
for the MoveBot API can be found in `movebot/docs`. 

For now, documentation is only viewable by building locally. To build the BotChocolate documentation,
navigate the to `docs/` directory and run `make html` (note you must have `make` installed).
There now should be a new directory `_build/html` and in it you will find a file, `index.html`
which you can open in your web browser of choice to view the documentation.

To view the MoveBot API documentation, navigate to `movebot/docs` and again run `make html` and the `index.html` 
file will be located at `movebot/docs/_build/html/index.html`

### Nodes
say nodes and what they do

### Launch Files
say launch files and what they launch

## The Group

![bot_choc-min](https://user-images.githubusercontent.com/46512429/206768445-4503edc2-2075-48b4-baf7-e6dc7bd3ca86.png)

