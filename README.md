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
*To install Numpy*
```
pip install numpy
```
*To install matplotlib*
```
pip install matplotlib
```
*To install python-csv*
```
pip install python-csv
```
*To install pandas*
```
pip install pandas
```

## Part01: 2D Implementation
*Downlaod the Part01_folder inside the zip file to your home directory*

```
cd Part01_folder/python3 proj3p2_naveeen_orlandis.py
```
*Input the start node coordinates, start point orintation, goal node coordinates, left wheel's rpm(value in range 3 to 6), right wheel's rpm(value in range 3 to 6) and the clearance of the robot(in cm)*

## Part02: Gazebo Visualization
*Downlaod the bot_astar(ROS package) inside your catkin workspace. It is assumed that the turtlebot package is already installed inside your workspace since we are calling the urdf files of the robot from the package turtlebot3_simulation folder*

```
cd ~/catkin_ws
catkin build
roslaunch bot_astar turtlebot3_control.py
```
```
*Input Parameters*
For visulaizing Turtlebot in Gazebo, run the script from Part01 with the following input parameters (all parameters are in cm):
    - start_point_x, start_point_y = (50,100) 
    - start point orientation = 0
    - goal_point_x, goal_point_y = (550,100) 
    - left wheel's rpm = 6
    - right wheel's rpm = 3
    - robot clearance = 15
Place the csv files generated inside ~/catkin_ws/src/bot_astar/data
and rebuild your package. Follow Part02: Gazebo Visulaization instructions to launch the file

*Note: For other parameters the turtlebot may not followe the optimal path*
```
