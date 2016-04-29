# move_fetch
Fetch robot benchmark for motion planning

## Requirements
* ROS indigo
* ROS fetch_gazebo package
 * install from source code, or
 * command-line install  
   $ sudo apt-get install ros-indigo-fetch-gazebo
* Gazebo for ROS
* Gazebo shelf model
 * Download at: http://amazonpickingchallenge.org/2015/gazebo_pod.shtml?
 * Install in ~/.gazebo/models

## Build
$ catkin_make
* If script is not built, make it executable  
  $ chmod +x scripts/prepare_simulated_robot.py

## Run benchmark
$ launch move_fetch apc.launch

## Modify shelf position
$ gedit worlds/apc.sdf

for example:
```xml
    <include>
      <uri>model://kiva_pod</uri>
      <pose>1 0 0 0 0 1.57</pose>
    </include>
```
