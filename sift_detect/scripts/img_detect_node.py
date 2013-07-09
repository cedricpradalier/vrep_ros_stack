#!/usr/bin/python

import roslib
roslib.load_manifest('sift_detect')

import sys
import os

import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Image
from sift_detect.msg import sift_keypoints_array
from sift_detect.msg import sift_keypoint

import cv
import cv2
import numpy as np
import itertools
from cv_bridge import CvBridge

from pylab import imread, imshow, gray, mean


from pylab import *


if __name__ == '__main__':


	pkgdir = roslib.packages.get_pkg_dir("opencv2")

	dirname, filename = os.path.split(os.path.abspath(__file__))

	#print "path",os.path

	path_imref = ""+dirname+"/../ROS/HeronLorraine.jpg"

	im_ref = cv2.imread(path_imref)

	if(im_ref == None):
		print "image vide"
	
	detector = cv2.FeatureDetector_create("SIFT")
	descriptor = cv2.DescriptorExtractor_create("SIFT")

	skp = detector.detect(im_ref)
	skp,sd = descriptor.compute(im_ref, skp)
	
	#print 'number of KeyPoint objects skp', len(skp)

	def detect_and_draw(imgmsg):

		global skp, sd
		global im_ref
		#print 'number of KeyPoint objects skp', len(skp)
		
	
		br = CvBridge()
		temp = br.imgmsg_to_cv(imgmsg, "bgr8")
		
		im_height = temp.height
		im_length = temp.width
	
		cv.ShowImage("view",temp)
	
		cv.WaitKey(10)
		template = np.asarray(temp)
	
	
		tkp = detector.detect(template)
		tkp,td = descriptor.compute(template, tkp)
	
		#print 'number of KeyPoint objects tkp', len(tkp)
		#print 'number of KeyPoint objects skp', len(skp)
	
		flann_params = dict(algorithm=1, trees=4)
		flann = cv2.flann_Index(sd, flann_params)
		idx, dist = flann.knnSearch(td, 1, params={})
		del flann

		dist = dist[:,0]/2500.0
		dist = dist.reshape(-1,).tolist()
		idx = idx.reshape(-1).tolist()
		indices = range(len(dist))
		indices.sort(key=lambda i: dist[i])
		dist = [dist[i] for i in indices]
		idx = [idx[i] for i in indices]

		skp_final = []
		for i, dis in itertools.izip(idx, dist):
			if dis < threshold:
				skp_final.append(skp[i])
			else:
				break
	
			
		tkp_final = []
		for i, dis in itertools.izip(range(len(idx)), dist):
			if dis < threshold:
				tkp_final.append(tkp[indices[i]])
			else:
				break
		
		h1, w1 = im_ref.shape[:2]
		h2, w2 = template.shape[:2]
		nWidth = w1+w2
		nHeight = max(h1, h2)
		hdif = (h1-h2)/2
		newimg = np.zeros((nHeight, nWidth, 3), np.uint8)
		newimg[hdif:hdif+h2, :w2] = template
		newimg[:h1, w2:w1+w2] = im_ref

		tkp_final
		skp_final

		#print 'number of KeyPoint objects in skp_final', len(skp_final)
		#print 'number of KeyPoint objects in tkp_final', len(tkp_final)

		for i in range(min(len(tkp), len(skp_final))):
			pt_a = (int(tkp_final[i].pt[0]), int(tkp_final[i].pt[1]+hdif))
			pt_b = (int(skp_final[i].pt[0]+w2), int(skp_final[i].pt[1]))
		
			cv2.circle(newimg, pt_a, int(tkp_final[i].size),(160,32,240),1)
			cv2.circle(newimg, pt_b, int(skp_final[i].size),(160,32,240),1)
			cv2.line(newimg, pt_a, pt_b, (255, 0, 0))
	
			cv.ShowImage("sift",cv.fromarray(newimg))
		
		kp_array = sift_keypoints_array()
		kp_array.skp = [sift_keypoint(*k.pt) for k in skp_final]
		kp_array.tkp = [sift_keypoint(*k.pt) for k in tkp_final]

			
		
		pk_pub.publish(kp_array)
		
		key=cv.WaitKey(10) & 0xFF
		
		if key == ord('d'):
			im_ref = template
			skp = tkp
			sd = td
			


	rospy.init_node('img_detect')
	threshold = rospy.get_param("~threshold",1)
	pk_pub = rospy.Publisher("~pk_pub",sift_keypoints_array)
	image_topic = "/vrep/visionSensor"
	rospy.Subscriber(image_topic, sensor_msgs.msg.Image, detect_and_draw)
	rospy.spin()

