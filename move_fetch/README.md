# move_fetch
Fetch robot benchmark for motion planning

## Requirements
* ROS indigo
* ROS fetch_gazebo package
 * install from source code, or
 * command-line install  
   $ sudo apt-get install ros-indigo-fetch-gazebo
* Moveit!
* Gazebo for ROS
* Gazebo shelf model
 * Download at: http://amazonpickingchallenge.org/2015/gazebo_pod.shtml?
 * Install in ~/.gazebo/models

## Build
$ catkin_make
* If script is not built, make it executable  
  $ chmod +x scripts/prepare_simulated_robot.py

## Run benchmark
* Run gazebo  
  $ roslaunch move_fetch apc_gazebo.launch

* Run apc_moveit.launch, which will run
 * map->odom tf broadcaster
 * *fetch_moveit_config/move_group.launch*
 * *rviz* with local config file  
   $ roslaunch move_fetch apc_moveit.launch
 * **Moveit envirnment is not set up**

## Modify shelf position in Gazebo
$ gedit worlds/apc.sdf

For example:
```xml
    <include>
      <uri>model://kiva_pod</uri>
      <pose>1 0 0 0 0 1.57</pose>
    </include>
```

## Modify environments in Moveit!
* Modify apc_moveit.launch
