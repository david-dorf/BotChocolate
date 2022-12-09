# BotVis
Computer Vision Package for BotChocolate

## Package Description
The bot_vis is consists of two executable, `april_tf.py` and `calibration.py`.

`april_tf.py` launches a node that listens to the transformations from the camera_link frame to the AprilTags of the kettle and jig. It then creates and broadcasts transformations to the kettle's button, the cocoa scooper, the cup, and the stirrer. Additionally, it publishes the positions of the scooper, kettle, kettle button, cup, stirrer, and jig to respective topics.

The `calibration.py` file can be run while the april_tf node is running to calibrate the robot to get the transformation from the camera_link to the base of the robot and save it in a yaml file to be used when running the entire process later on.

## Operation
The `april_tf` node must run concurrently with Christian Rauch's apriltag_ros node (packages link in VCS file) and the realsense_ros node. Both of these nodes are launched in the 
`launch_vision.py` launch file.

To calibrate the system, the end-effector AprilTag must be placed in the robot's gripper, aligned with the panda_link_tcp frame, and in the field of view of the camera. Next, just call the launch launch file with the calibration flag.

### Call Launch File
For april_tf:
`ros2 launch bot_vis launch_vision.py`

For calibration:
`ros2 launch bot_vis launch_vision.py calibration:=true`
