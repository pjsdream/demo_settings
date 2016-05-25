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
* ITOMP
 * Clone https://github.com/Chpark/itomp_ca_planner.git (branch: point_cloud_fetch) and build in rosbuild workspace
 * ITOMP is rosbuild package and move_fetch is catkin package. To resolve this inconsistency, refer to http://wiki.ros.org/catkin/Tutorials/using_rosbuild_with_catkin
 
## Build
$ catkin_make
* If script is not built, make it executable  
  $ chmod +x scripts/prepare_simulated_robot.py

## Run table benchmark
1. Launch MoveIt! table demo
 * Run with gazebo  
  $ roslaunch move_fetch table.launch use_gazebo:=true
 * Run without gazebo  
   $ roslaunch move_fetch table.launch
2. Execute motion planning node  
  $ rosrun move_fetch move_fetch

========================================================

# Under development

## Requirements
* Gazebo shelf model
 * Download at: http://amazonpickingchallenge.org/2015/gazebo_pod.shtml?
 * Install in ~/.gazebo/models

## Run Amazon Picking Challenge benchmark
* Run with gazebo  
  $ roslaunch move_fetch apc_gazebo.launch

* Run apc_moveit.launch, which will run
 * map->odom tf broadcaster
 * *fetch_moveit_config/move_group.launch*
 * *rviz* with local config file  
   $ roslaunch move_fetch apc_moveit.launch
 * **Moveit envirnment is not set up**

* Modify shelf position in Gazebo  
  $ gedit worlds/apc.sdf
 * For example:
```xml
    <include>
      <uri>model://kiva_pod</uri>
      <pose>1 0 0 0 0 1.57</pose>
    </include>
```

* Modify environments in Moveit!
 * Modify apc_moveit.launch
