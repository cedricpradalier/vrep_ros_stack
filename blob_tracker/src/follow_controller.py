#!/usr/bin/env python
import roslib
roslib.load_manifest('blob_tracker')

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
        while (not rospy.is_shutdown()) and ((not self.info) or (not self.blob)):
            self.pub.publish(t)
            rospy.sleep(0.1)
        while not rospy.is_shutdown():
            x_blob = self.blob.x_offset + self.blob.width/2.0
            y_blob = self.blob.y_offset + self.blob.height/2.0
            x_center = self.info.width / 2.0
            y_setpoint = 0.65 * self.info.height
            # rospy.loginfo("Set point: %.1f %.1f Current %.1f %.1f" % (x_center,
            #     y_setpoint, x_blob, y_blob))
            t = Twist()
            t.linear.x = -0.03 * (y_blob - y_setpoint)
            t.angular.z = -0.005 * (x_blob - x_center)
            self.pub.publish(t)

if __name__=="__main__":
    demo = BlobFollower()
    demo.run()
