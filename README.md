# Omnidirectional AGV with integrated low level controlller                                                                                                                                                          

## Instructions

1. Clone and build the repository in local workspace.
2. Launch the gazebo simulation of the agv in the industrial_shop_floor world:
   Add the following lines in your bashrc:
```shell
export GAZEBO_MODEL_PATH=$(rospack find agv_low)/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$(rospack find agv_low)/worlds:$GAZEBO_RESOURCE_PATH
```
And launch the simulation:
```shell
roslaunch agv_low agv_low_youbot_collisionwheel.launch
```
3. Launch the laser_scan_merger node to merge the laser scans:
```shell
roslaunch scan_merger laserscan_multi_merger.launch
```
4. Launch move_base for agv:
```shell
roslaunch rover_navigation move_base.launch
```
5. To launch to position controller :
```shell
roslaunch position_controller start.launch
```
6. To publish the desired pose :
```shell
rostopic pub /agv_mecanum/sp_pose geometry_msgs/PoseStamped "header: 
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 1.0 
    y: 1.0 
    z: 0.0 
  orientation:
    x: 0.0 
    y: 0.0 
    z: 0.99749499
    w: 0.0707372"
```
7. To execute a sample trajectory :
```shell
rosrun position_controller trajectory_planner.py
```

