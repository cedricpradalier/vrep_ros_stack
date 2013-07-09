#!/usr/bin/env python

"""
#include <ros/ros.h>
#include <turtlesim/Velocity.h>
#include <sensor_msgs/Joy.h>
"""

import roslib; roslib.load_manifest('test_joy')
import rospy
#from turtlesim.msg import Velocity
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

"""
0 A
1 B
2 X
3 Y
4 LB
5 RB
7 start
8 power
10 cross key up
11 cross key down
12 cross key left
13 cross key right
14 back
"""

class Controler:
	def __init__(self):
		rospy.init_node('joy_test', anonymous=True)
		self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback)
		self.left_pub = rospy.Publisher('vrep/leftWheelCommand', Float64)
		self.right_pub = rospy.Publisher('vrep/rightWheelCommand', Float64)
		self.twist_pub = rospy.Publisher('vrep/twistCommand', Twist)
		#self.vel_val = Velocity()
		self.right_val = Float64(0)
		self.left_val = Float64(0)
		#self.wheel_val = Float64(0)
		
		self.twist = Twist()
		self.twist.linear = Vector3(0,0,0)
		self.twist.angular = Vector3(0,0,0)

		self.button_A = 0
		self.button_B = 1
		self.button_LB = 4
		self.button_RB = 5

		self.left_joy_x = 0
		self.right_joy_x = 3
		self.left_joy_y = 1
		self.right_joy_y = 4

		#self.linear = 1
		#self.angular = 0
		self.scale = 1

		self.timer = 0

	def start(self):
		while not rospy.is_shutdown():
			self.timer = self.timer + 0.1
			if self.timer >= 3:
				self.twist = Twist()
			rospy.loginfo("linear : " + str(self.twist.linear.x) + " ; " + str(self.twist.linear.z))
			rospy.loginfo("angular : " + str(self.twist.angular.x) + " ; " + str(self.twist.angular.z))
			#rospy.loginfo(rospy.get_name() + " : " + str(self.right_val) + " ; " + str(self.left_val))
			#self.right_pub.publish(self.right_val)
			#self.left_pub.publish(self.left_val)
			self.twist_pub.publish(self.twist)
			rospy.sleep(0.1)

	def joyCallback(self, joy):
		#self.right_val = self.scale*joy.axes[self.right_joy_y]
		#self.left_val = self.scale*joy.axes[self.left_joy_y]
		self.timer = 0
		if joy.buttons[self.button_LB] > 0:
			self.scale = max(0.1,self.scale-0.2)
		elif joy.buttons[self.button_RB] > 0:
			self.scale = min(2,self.scale+0.2)

		self.twist.linear.x = self.scale*joy.axes[self.left_joy_y]
		self.twist.angular.z = self.scale*joy.axes[self.right_joy_x]
		
		#self.vel_val.linear = self.l_scale*joy.axes[self.linear]
		#self.vel_val.angular = self.a_scale*joy.axes[self.angular]
		

if __name__ == '__main__':
	c = Controler()
	c.start()





