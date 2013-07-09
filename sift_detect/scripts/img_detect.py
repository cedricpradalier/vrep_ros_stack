#!/usr/bin/python


import roslib
roslib.load_manifest('sift_detect')

import sys
import os

import rospy
import cv
import cv2
import numpy as np
import itertools

from pylab import imread, imshow, gray, mean

from pylab import *



if __name__ == '__main__':
	
	key = ''

	pkgdir = roslib.packages.get_pkg_dir("opencv2")
	
	dirname, filename = os.path.split(os.path.abspath(__file__))
	
	#print "path",os.path

	path_imref = ""+dirname+"/../ROS/HeronLorraine.jpg"
	path_template = ""+dirname+"/../ROS/HeronLorraine-ScaledPerspective.jpg"

	im_ref = cv2.imread(path_imref)
	
	if(im_ref == None):
		print "image vide"
		
	template = cv2.imread(path_template)

	
	detector = cv2.FeatureDetector_create("SIFT")
	descriptor = cv2.DescriptorExtractor_create("SIFT")

	skp = detector.detect(im_ref)
	skp,sd = descriptor.compute(im_ref, skp)

	tkp = detector.detect(template)
	tkp,td = descriptor.compute(template, tkp)


	print 'number of KeyPoint objects', len(skp)
	print 'number of KeyPoint objects', len(tkp)

	############################ALGO##############################

	flann_params = dict(algorithm=1, trees=4)
	flann = cv2.flann_Index(sd, flann_params)
	idx, dist = flann.knnSearch(td, 1, params={})
	del flann

	for i in range(len(skp)):
		pt = (int(skp[i].pt[0]), int(skp[i].pt[1]))
		plot(skp[i].pt[0],skp[i].pt[1],'ob')

	
	dist = dist[:,0]/2500.0
	dist = dist.reshape(-1,).tolist()
	idx = idx.reshape(-1).tolist()
	indices = range(len(dist))
	indices.sort(key=lambda i: dist[i])
	dist = [dist[i] for i in indices]
	idx = [idx[i] for i in indices]

	distance = 1

	skp_final = []
	for i, dis in itertools.izip(idx, dist):
		if dis < distance:
			skp_final.append(skp[i])
		else:
			break
		
		    
	tkp_final = []
	for i, dis in itertools.izip(range(len(idx)), dist):
		if dis < distance:
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
	
	tkp = tkp_final
	skp = skp_final
	
	print 'number of KeyPoint objects', len(skp)
	print 'number of KeyPoint objects', len(tkp)
	
	for i in range(min(len(tkp), len(skp))):
		pt_a = (int(tkp[i].pt[0]), int(tkp[i].pt[1]+hdif))
		pt_b = (int(skp[i].pt[0]+w2), int(skp[i].pt[1]))
		cv2.line(newimg, pt_a, pt_b, (255, 0, 0))

	############################ALGO##############################

	while key != ord('q') and key != ord('Q'):
		
		cv.ShowImage("sift",cv.fromarray(newimg))

		key=cv.WaitKey(10) & 0xFF
	
		

