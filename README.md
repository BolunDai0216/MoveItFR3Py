# MoveItFR3Py

This repo contains code developed for running MoveIt on the Franka Research 3 (FR3) robot.

## Basic Usage

First run 

```console
roslaunch panda_moveit_config franka_control.launch robot_ip:=<fci-ip> load_gripper:=<true|false> 
```

Then, run

```console
rosrun moveit_fr3_py franka_pick_n_place.py
```