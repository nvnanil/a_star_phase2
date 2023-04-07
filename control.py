#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import random

def generate_velocities(min_linear, max_linear, min_angular, max_angular):
    linear_velocity = random.uniform(min_linear, max_linear)
    angular_velocity = random.uniform(min_angular, max_angular)
    return (linear_velocity, angular_velocity)

class VelocityPublisher:
    def __init__(self, min_linear, max_linear, min_angular, max_angular):
        self.min_linear = min_linear
        self.max_linear = max_linear
        self.min_angular = min_angular
        self.max_angular = max_angular

        rospy.init_node('velocity_publisher')
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def publish_velocities(self):
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            linear, angular = generate_velocities(self.min_linear, self.max_linear, self.min_angular, self.max_angular)
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self.publisher.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    min_linear = 0.1
    max_linear = 0.5
    min_angular = -1.0
    max_angular = 1.0

    velocity_publisher = VelocityPublisher(min_linear, max_linear, min_angular, max_angular)
    velocity_publisher.publish_velocities()






