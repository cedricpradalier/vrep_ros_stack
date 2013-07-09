#!/usr/bin/env python
import roslib
roslib.load_manifest('floor_mapper')

import rospy
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2.cv as cv
from cv_bridge import CvBridge
import math
import tf
import numpy

class FloorMapper:
    def __init__(self):
        self.proba = None
        self.info = None
        self.br = CvBridge()
        rospy.init_node('floor_projector')

        image_size = rospy.get_param("~floor_size_pix",1000)
        image_extent = rospy.get_param("~floor_size_meter",5.0)
        self.target_frame = rospy.get_param("~target_frame","/body")
        self.horizon_offset = rospy.get_param("~horizon_offset_pix",20)

        self.floor_map = cv.CreateImage( (image_size,image_size), 8, 1)
        self.x_floor = 0.0
        self.y_floor = self.floor_map.height / 2.0
        self.floor_scale = self.floor_map.width / image_extent

        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher("~floor",Image)
        rospy.Subscriber("~probabilities",Image,self.store_proba)
        rospy.Subscriber("~info",CameraInfo,self.store_info)
        rospy.loginfo("Waiting for first proba and camera info")
        while (not rospy.is_shutdown()) and ((not self.info) or (not self.proba)):
            rospy.sleep(0.1)

    def store_proba(self,proba):
        # print "Got Image"
        if not self.info:
            return
        # print "Processing"
        self.timestamp = proba.header.stamp
        I = self.br.imgmsg_to_cv(proba,"8UC1")
        self.proba = cv.CloneMat(I)
        cv.Threshold(I,self.proba,0xFE,0xFE,cv.CV_THRESH_TRUNC)
        try:
            # (trans,rot) = self.listener.lookupTransform(proba.header.frame_id, '/world', proba.header.stamp)
            self.listener.waitForTransform(proba.header.frame_id,self.target_frame,proba.header.stamp,rospy.Duration(1.0))
            trans = numpy.mat(self.listener.asMatrix(self.target_frame,proba.header))
            # print "Transformation"
            # print trans
            dstdir = [trans * v for v in self.dirpts3d]
            # print "Destination dir"
            # print dstdir
            origin = trans * self.origin
            origin = origin / origin[3,0]
            # origin = numpy.matrix([0.0, 0.0, origin[2,0] / origin[3,0], 1.0]).T
            # print "Origin"
            # print origin

            self.dstpts2d = cv.CreateMat(4,2,cv.CV_32F)
            for i in range(4):
                self.dstpts2d[i,0] = self.x_floor + (origin[0,0] - dstdir[i][0,0]*origin[2,0]/dstdir[i][2,0])*self.floor_scale
                self.dstpts2d[i,1] = self.y_floor - (origin[1,0] - dstdir[i][1,0]*origin[2,0]/dstdir[i][2,0])*self.floor_scale
            # print numpy.asarray(self.dstpts2d)

            # print "Source points"
            # print numpy.asarray(self.srcpts2d)
            # print "Dest points"
            # print numpy.asarray(self.dstpts2d)
            self.H = cv.CreateMat(3,3,cv.CV_32F)
            cv.FindHomography(self.srcpts2d,self.dstpts2d,self.H)
            # print "Homography"
            # print numpy.asarray(self.H)

            cv.WarpPerspective(cv.GetSubRect(self.proba,(0,self.horizon_offset,self.proba.width,self.proba.height-self.horizon_offset)),
                    self.floor_map,self.H, flags=cv.CV_INTER_NN+cv.CV_WARP_FILL_OUTLIERS , fillval=0xFF)

            msg = self.br.cv_to_imgmsg(self.floor_map)
            msg.header.stamp = proba.header.stamp
            msg.header.frame_id = self.target_frame
            self.pub.publish(msg)
            # print "Publishing image"
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Exception while looking for transform"
            return
        
        
            

    def store_info(self,info):
        if not self.info:
            # assuming no distortion
            self.f = info.K[0]
            self.xc = info.K[2]
            self.yc = info.K[5]
            print("Got camera info: f %.2f C %.2f %.2f" % (self.f,self.xc,self.yc))
            self.origin = numpy.zeros((4,1))
            self.origin[3,0] = 1.0

            self.horizon_offset = int(math.ceil(info.height/2. + self.horizon_offset))
            srcpts = [[0,info.height-1],[info.width-1,info.height-1],\
                    [0,self.horizon_offset], [info.width-1,self.horizon_offset]]
            self.srcpts2d = cv.CreateMat(4,2,cv.CV_32F)
            for i in range(4):
                self.srcpts2d[i,0] = srcpts[i][0]
                self.srcpts2d[i,1] = srcpts[i][1] - self.horizon_offset
            self.dirpts3d = []
            for i in range(4):
                v3 = numpy.matrix([-(srcpts[i][0]-self.xc) / self.f, 
                    -(srcpts[i][1]-self.yc) / self.f, 1.0, 0.0]).T
                n = math.sqrt((v3.T*v3).sum())
                self.dirpts3d.append(v3/n)
            self.info = info
        # print self.dirpts3d

    def run(self):
        rospy.loginfo("Starting floor projection")
        rospy.spin()

if __name__=="__main__":
    demo = FloorMapper()
    demo.run()

