# TECHNICAL PROJECT - Project 1

This repository contains the implementation of the technical projects for the courses of Robotics Lab (RL) and Field and Service Robotics (FSR) for the academic year 2021.

The assigned [project 1](https://prisma.dieti.unina.it/images/Courses/FSR/FSR-RL-TP-Assignments.pdf) required to develop a control system for a wheeled mobile robot used in a logistic environment.

## Brief Packages Description

- **material** : contains the report, images and videos of the project.
- **turtlebot3** : contains the packages used for the robot description (URDF) together with other packages that implements SLAM (gmapping), localization (amcl) and the joystick simulation (teleop).
- **turtlebot3_simulations**: contains the stuff needed for the simulation of both the robot and the environment in gazebo.
- **ar_marker_test** and **aruco_ros** : are used to work with Aruco AR Marker. 
In the first phase it's used to locate the AR Markers in the map.
In the second phase it's used to read the AR Markers ID.
- **planner** : implementation of RRT with A*.
- **mobile_navigation** : implementation of the controller (I/O FBL) and also of the service routine to adjust in front of the AR Marker (positioning_routine).
- **logistic_task** : implementation of the mission in form of state machine. 
- **trajectory_tracking** : launch a python script to plot the desired variables.

## Dependecies

- ROS Noetic
- Catkin
- RViz
- Gazebo
- Eigen
- gmapping
- amcl
- Turtlebot3
- [aruco_ros](https://github.com/pal-robotics/aruco_ros) - Aruco libraries to read ARMarkers

## Installation

Clone the repository in the src folder of a catkin workspace. 

```bash
cd [workspace]/src
git clone https://github.com/s-contento/TP.git
cd ..
catkin_make
source devel/setup.bash
```

## Usage

The mission is completed in 2 phases:

- Phase 1: creation of the map with SLAM.

For this task you have to load the models in the simulation scene and move the turtlebot in it thorugh the teleoperated package.


```bash
# launch world, gmapping, aruco and key_teleop packages
roslaunch logistic_task phase1.launch
```

![ezgif com-gif-maker](https://user-images.githubusercontent.com/80551374/134597249-642f732e-9f43-440e-9aba-7000d07ad754.gif)
- Phase 2: localization and navigation.

In this second phase the task is to localize the robot and start the logistic task that will bring the robot in the warehouse, search and read for the AR Marker and finally go to the desired room on the basis of the code on the Marker.

```bash
# launch world, amcl, aruco, planner, controller, logistic task package
roslaunch logistic_task phase2.launch
```
![ezgif com-gif-maker (1)](https://user-images.githubusercontent.com/80551374/134597379-1a2a2a68-69d8-4faf-bae1-c058e35f9227.gif)

