# MoveItFR3Py

This repo contains code developed for running MoveIt on the Franka Research 3 (FR3) robot.

## Basic Usage

First launch `franka_control` using the command 

```console
roslaunch panda_moveit_config franka_control.launch robot_ip:=<fci-ip> load_gripper:=<true|false> 
```

Then, run the python script

```console
rosrun moveit_fr3_py franka_pick_n_place.py
```

## Install `franka_ros` and `MoveIt` using `catkin build`

To install both `franka_ros` and `MoveIt` using `catkin build`, one needs to first download both `franka_ros` and `MoveIt` into the `src` folder of a catkin workspace. Then run the commands

```console
cd ~/ws_moveit
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
catkin build
```