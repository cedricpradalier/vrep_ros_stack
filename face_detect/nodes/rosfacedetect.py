#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
"""
import roslib
roslib.load_manifest('face_detect')

import sys
import os
from optparse import OptionParser

import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv
from face_detect.msg import *
from sensor_msgs.msg import *

# Parameters for haar detection
# From the API:
# The default parameters (scale_factor=2, min_neighbors=3, flags=0) are tuned 
# for accurate yet slow object detection. For a faster operation on real video 
# images the settings are: 
# scale_factor=1.2, min_neighbors=2, flags=CV_HAAR_DO_CANNY_PRUNING, 
# min_size=<minimum possible face size

min_size = (10, 10)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0
display = True

if __name__ == '__main__':

    # pkgdir = roslib.packages.get_pkg_dir("opencv2")
    haarfile = "/opt/ros/fuerte/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml"

    parser = OptionParser(usage = "usage: %prog [options] [filename|camera_index]")
    parser.add_option("-c", "--cascade", action="store", dest="cascade", type="str", help="Haar cascade file, default %default", default = haarfile)
    (options, args) = parser.parse_args()

    cascade = cv.Load(options.cascade)
    br = CvBridge()
    rospy.init_node('facedetect')

    display = rospy.get_param("~display",True)
    rois_pub = rospy.Publisher('~rois', ROIArray)

    def detect_and_draw(imgmsg):
        img = br.imgmsg_to_cv(imgmsg, "bgr8")
        # allocate temporary images
        gray = cv.CreateImage((img.width,img.height), 8, 1)
        small_img = cv.CreateImage((cv.Round(img.width / image_scale),
                       cv.Round (img.height / image_scale)), 8, 1)

        # convert color input image to grayscale
        cv.CvtColor(img, gray, cv.CV_BGR2GRAY)

        # scale input image for faster processing
        cv.Resize(gray, small_img, cv.CV_INTER_LINEAR)

        cv.EqualizeHist(small_img, small_img)

        rois = ROIArray()
        rois.header = imgmsg.header

        if(cascade):
            faces = cv.HaarDetectObjects(small_img, cascade, cv.CreateMemStorage(0),
                                         haar_scale, min_neighbors, haar_flags, min_size)
            if faces:
                for ((x, y, w, h), n) in faces:
                    # the input to cv.HaarDetectObjects was resized, so scale the 
                    # bounding box of each face and convert it to two CvPoints
                    pt1 = (int(x * image_scale), int(y * image_scale))
                    pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
                    cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)					
                    region_of_interest = RegionOfInterest()
                    region_of_interest.x_offset = x * image_scale
                    region_of_interest.y_offset = y * image_scale
                    region_of_interest.width = w * image_scale
                    region_of_interest.height = h * image_scale
                    rois.rois.append(region_of_interest);
                rospy.loginfo("%d Face found!" % len(faces))

        rois_pub.publish(rois)        
        if display:
            cv.ShowImage("result", img)
            cv.WaitKey(6)

    rospy.Subscriber("~image", sensor_msgs.msg.Image, detect_and_draw)
    rospy.spin()
