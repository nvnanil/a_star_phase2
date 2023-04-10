# ENPM661 - Planning for Autonomous Robots - Project 3 Phase 2
```
Name: Naveen Anil
UID: 119398593
Directory ID: nvnanil
Name: Orlandis D. Smith
UID: 1184933077
Directory ID: osmith15
```
Course:  ENPM661 - Planning for Autonomous Robots

## Software Requirements
*Software Requirements*
```
Ubuntu 20.04
ROS Noetic
Python 3
Turtlebot3_Simulation package: git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

```
*Dependencies*
```
Ubuntu 20.04
ROS Noetic
Python 3
```

## Part01: 2D Implementation*
*Downlaod the Part01_folder inside the zip file to your home directory*

```
cd Part01_folder/python3 proj3p2_naveeen_orlandis.py
```
*Input the start node coordinates, start point orintation, goal node coordinates, left wheel's rpm(value in range 3 to 6), right wheel's rpm(value in range 3 to 6) and the clearance of the robot(in cm)*

## Part02: Gazebo Visualization
*Downlaod the bot_astar(ROS package) inside your catkin workspace*

```
cd catkin_ws
catkin build
roslaunch bot_astar turtlebot3_control.py
```
