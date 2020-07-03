# hector_quadrotor_navigation
This project provides an implementation of 3D navigation for the hector quadrotor using the Moveit! Motion Planning Framework.

## Support
This package is developed on Ubuntu 18.04 with [ROS melodic] (http://wiki.ros.org/Installation/Ubuntu). The support for other versions of ROS is under further development.

## Getting Started
To install the necessary packages:

```Shell
./setup.bash
```

To build the ros package: 

```Shell
catkin build
```

## Hector Quadrotor Navigation
To launch the simulation environment : 
```Shell
roslaunch hector_navigation_simulation empty_world_navigation.launch
```
Users can custom the simulation environment by providing gazebo world files.

To launch the navigation stack:
```Shell
roslaunch hector_navigation_simulation hector_navigation.launch
```

This package currently provides two ros service:

### Takeoff service
```Shell
rosservice call /hector_takeoff "takeoff_distance_m: Desired Takeoff distance"
```
The user is able to set the hovering height for takeoff.

### Navigation service
```Shell
rosservice call /hector_navigation "goal:
  x: 0.0
  y: 0.0
  z: 0.0
speed: 0.0"
```
Users are able to set the navigation goal position with a desired cruise speed.

## Acknowledge

This project is developed heavily referring to Tahsincan Köse's [hector-moveit](https://github.com/tahsinkose/hector-moveit
) project.
