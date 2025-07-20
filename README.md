<div align="center">
  <img src="/media/Test.gif" alt="GIF of test" width="640"/>
</div>

# Introduction
This repo is a ROS2-based project for teleoperating two robotic arms through camera-based pose detection.

# Installation
Install the following dependencies for this package:
- `Ubuntu 22.04`
- [`ROS2 Humble`](https://docs.ros.org/en/humble/Installation.html)
- [`mediapipe`](https://ai.google.dev/edge/mediapipe/solutions/setup_python) for python (can also be installed using rosdep)
- [`MoveIt2`](https://moveit.ai/install-moveit2/binary/) (can also be installed using rosdep)

Git clone or download this repo to your PC, launch the terminal and move to the root of the workspace (`robot_arms_teleop\`). Run the following commands to install any remanining dependencies:
```
sudo rosdep init
rosdep install --from-paths src -y --ignore-src
```
Then run the following to build the package files:
```
colcon build
```
# Running
First you nned to source the installed files using:
```
source install/setup.bash
```
You need to individually run the three launch files in separate terminals:
```
ros2 launch panda_moveit_config demo.launch.py
ros2 launch motion_planning motion_planning.launch.py
```
Wait for the robot_arms to spawn in the rviz viewport and run the last launch file:
```
ros2 launch goal_pose_publisher goal_pose_publisher.launch.py
```
You can check the available options for the goal_publisher launch file using `--show-args`. Most important options are:
- `mode`: Options are `VIDEO` and `LIVE_STREAM` (default). `VIDEO` mode works in conjunction with the `video_file` parameter.
- `video_file`: Name of the video file in `src/goal_pose_publisher/video/`. Defaults to `Video.mp4`. This option only works when `mode` is `VIDEO`.

# Issues
The motion_planning node can crash if you have a slow PC due to mismatch between frames.