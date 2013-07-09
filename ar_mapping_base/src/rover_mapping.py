#!/usr/bin/env python
import roslib; roslib.load_manifest('ar_mapping_base')
import rospy
from geometry_msgs.msg import PointStamped
import tf
from tf.transformations import euler_from_quaternion
import numpy

from mapping_kf import *


from ar_track_alvar.msg import AlvarMarkers


class RoverMapping:
    def __init__(self,name):
        self.name = name
        self.encoder_precision = 0.05 # [m]
        self.ar_precision = 0.50 # [m]
        self.target_frame = "/world"
        rospy.init_node('rover_mapping')
        self.name = rospy.get_param("~rover_name",self.name)
        self.target_frame = rospy.get_param("~target_frame",self.target_frame)
        self.ar_precision = rospy.get_param("~ar_precision",self.ar_precision)
        rospy.loginfo("Starting rover driver for rover '%s' " % (self.name))
        self.last_cmd = rospy.Time.now()
        self.listener = tf.TransformListener()
        self.connected = False

        # Instantiate the right filter based on launch parameters
        self.mapper = MappingKF()

    def ar_cb(self, markers):
        for m in markers.markers:
            self.listener.waitForTransform(self.target_frame,'/%s/ground'% self.name, m.header.stamp, rospy.Duration(1.0))
            self.listener.waitForTransform("/%s/ground"%self.name,m.header.frame_id, m.header.stamp, rospy.Duration(1.0))
            ((x,y,z),rot) = self.listener.lookupTransform(self.target_frame,'/%s/ground'%self.name, m.header.stamp)
            euler = euler_from_quaternion(rot)
            X = vstack([x,y,euler[2]])
            m_pose = PointStamped()
            m_pose.header = m.header
            m_pose.point = m.pose.pose.position
            m_pose = self.listener.transformPoint("/%s/ground"%self.name,m_pose)
            Z = vstack([m_pose.point.x,m_pose.point.y])
            self.mapper.update_ar(Z,X,m.id,self.ar_precision)
        self.mapper.publish(self.target_frame,markers.header.stamp)


    def run(self):
        timeout = True
        rate = rospy.Rate(2)
        rospy.sleep(1.0)
        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_cb)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        rd = RoverMapping("rover") 
        rd.run()
    except rospy.ROSInterruptException:
        pass
