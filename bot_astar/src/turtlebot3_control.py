#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import pandas as pd
import time

#Robot specifications (in meters)
r = 0.033
L = 0.16

def velocity_publisher():
	#Node initialization
	rospy.init_node('turtlebot3_velocity')
	rate = rospy.Rate(1/3)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
	m = 0
	time.sleep(2) 

	try:
		#Reading the CSV file
		data = pd.read_csv("~/catkin_ws/src/bot_astar/data/t_rpm.csv", delimiter=',', header=None)
		
		while not rospy.is_shutdown() and m < len(data):
			# Scaling the rpms
			l_rpm = data.loc[[m]][0]*0.45
			r_rpm = data.loc[[m]][1]*0.45

			#Calculating linear and angular velocities
			linear_velocity = (r*(l_rpm + r_rpm))/2.0 
			angular_velocity = r/L*(r_rpm - l_rpm)

			#Publishing the message to the rostopic
			robot_vel = Twist()
			robot_vel.linear.x = linear_velocity
			robot_vel.angular.z = angular_velocity
			pub.publish(robot_vel)
			rate.sleep()
			m = m+1

	except:
		print("Exception error")

	finally: #Stopping the robot
		robot_vel = Twist()
		robot_vel.linear.x = 0.0
		robot_vel.linear.y = 0.0
		robot_vel.linear.z = 0.0
		robot_vel.angular.x = 0.0
		robot_vel.angular.y = 0.0
		robot_vel.angular.z = 0.0
		pub.publish(robot_vel)
		print("***Robot stopped****")

if __name__=="__main__":
	try:
		velocity_publisher()

	except rospy.ROSInterruptException():
		pass