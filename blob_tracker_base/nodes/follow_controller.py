#!/usr/bin/env python
import roslib
roslib.load_manifest('blob_tracker_base')

import rospy
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist

class BlobFollower:
    def __init__(self):
        self.blob = None
        self.info = None
        rospy.init_node('blob_follow')
        self.pub = rospy.Publisher("~twistOut",Twist)
        rospy.Subscriber("~blob",RegionOfInterest,self.store_blob)
        rospy.Subscriber("~info",CameraInfo,self.store_info)

    def store_blob(self,blob):
        self.blob = blob

    def store_info(self,info):
        self.info = info

    def run(self):
        rospy.loginfo("Waiting for first blob and camera info")
        t = Twist()
        rate = rospy.Rate(10)
        while (not rospy.is_shutdown()) and ((not self.info) or (not self.blob)):
            self.pub.publish(t)
            rate.sleep()
        while not rospy.is_shutdown():
            self.pub.publish(t)
            rate.sleep()

if __name__=="__main__":
    demo = BlobFollower()
    demo.run()
