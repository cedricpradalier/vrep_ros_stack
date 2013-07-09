#!/usr/bin/python

import roslib
roslib.load_manifest('sift_detect')

import sys
import os

import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from sift_detect.msg import sift_keypoints_array
from sift_detect.msg import sift_keypoint

import cv
import cv2
import numpy as np
import itertools
from cv_bridge import CvBridge

from pylab import imread, imshow, gray, mean


from pylab import *



class VisualServo:
	def __init__(self):

		rospy.init_node('img_detect')
		self.scale = rospy.get_param("~scale",1)
		self.twist_pub = rospy.Publisher("~twist_pub",Twist)
		rospy.Subscriber("~info",CameraInfo,self.store_info)
		rospy.Subscriber("~kp", sift_keypoints_array, self.visual_servo)

	
	def visual_servo(self,kPoints):
		
		
		# kPoints.skp=[sift_keypoint(0,0), sift_keypoint(0,10), sift_keypoint(10,0)]
		# kPoints.tkp=[sift_keypoint(20,20), sift_keypoint(20,30), sift_keypoint(30,20)]
		
		nKP = len(kPoints.tkp)
		#print("************ len nkp = "+str(nKP))
		twist = Twist()
		error = np.asmatrix(np.zeros((2*nKP,1)))
		
		L = np.asmatrix(np.zeros((2*nKP,3)))
		
		for i in range(nKP):
			
			#print ("-------------- i="+str(i))
			# filling up the interaction matrix
			
			x = (-kPoints.tkp[i].x + self.xc)/self.f
			y = (-kPoints.tkp[i].y + self.yc)/self.f
			
			
			L[2*i,0] = -1
			L[2*i,1] = x
			L[2*i,2] = -(1+x*x)
			
			L[2*i+1,0] = 0
			L[2*i+1,1] = y
			L[2*i+1,2] = -x*y
			
			#L[2*i,0] = -1
			#L[2*i,1] = 0
			#L[2*i,2] = x
			#L[2*i,3] = x*y
			#L[2*i,4] = -(1+x*x)
			#L[2*i,5] = y
			
			#L[2*i+1,0] = 0
			#L[2*i+1,1] = -1
			#L[2*i+1,2] = y
			#L[2*i+1,3] = 1 + y*y
			#L[2*i+1,4] = -x*y
			#L[2*i+1,5] = -x
			
			# computing the error matrix
			
			error[2*i,0]=(-kPoints.tkp[i].x + self.xc)/self.f - (-kPoints.skp[i].x + self.xc)/self.f
			error[2*i+1,0]=(-kPoints.tkp[i].y + self.yc)/self.f - (-kPoints.skp[i].y + self.yc)/self.f

			
		L_pi = linalg.pinv(L)
		
		vel = -self.scale * L_pi*error
		
		t = Twist()
		
		#vc = [vx vz wy] = [ros.vy ros.vx ros.wz]

		t.linear.x = vel[1,0]
		t.linear.y = vel[0,0]
		t.linear.z = 0
		t.angular.x = 0
		t.angular.y = 0
		t.angular.z = vel[2,0]
		
		#print("twist command from servo: vy %.2f vz %.2f wx %.2f" % (t.linear.y,t.linear.z,t.angular.x))
		
		
		self.twist_pub.publish(t)	
		
	def store_info(self,info):
		self.f = info.K[0]
		self.xc = info.K[2]
		self.yc = info.K[5]
		
		#print("Got camera info: f %.2f C %.2f %.2f" % (self.f,self.xc,self.yc))
		

	def run(self):
		rospy.loginfo("Starting visual servo")
		rospy.spin()
		

if __name__ == '__main__':

	demo = VisualServo()
	demo.run()	
