#!/usr/bin/env python
import roslib; roslib.load_manifest('ar_slam_base')
import rospy
from std_msgs.msg import Float64,Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import tf
import numpy
import message_filters

from mapping_kf import *


from ar_track_alvar.msg import AlvarMarkers


class RoverMapping:
    def __init__(self,name):
        self.name = name
        self.encoder_precision = 0.05 # [m]
        self.ar_precision = 0.50 # [m]
        self.compass_precision = 10. * pi/180. # [rad]
        self.target_frame = "/world"
        self.ignore_id = False
        self.initial_x = -5.0
        self.initial_y = 2.5
        self.initial_theta = -pi/4
        self.encoder_precision = 0.05 # [m]
        rospy.init_node('rover_mapping')
        self.name = rospy.get_param("~rover_name",self.name)
        self.ignore_id = rospy.get_param("~ignore_id",self.ignore_id)
        self.target_frame = rospy.get_param("~target_frame",self.target_frame)
        self.ar_precision = rospy.get_param("~ar_precision",self.ar_precision)
        self.compass_precision = rospy.get_param("~compass_precision",self.compass_precision)
        self.encoder_precision = rospy.get_param("~encoder_precision",self.encoder_precision)
        self.initial_x = rospy.get_param("~initial_x",self.initial_x)
        self.initial_y = rospy.get_param("~initial_y",self.initial_y)
        self.initial_theta = rospy.get_param("~initial_theta",self.initial_theta)
        rospy.loginfo("Starting rover driver for rover '%s'" % (self.name))
        self.last_cmd = rospy.Time.now()
        self.listener = tf.TransformListener()
        self.ready = False
        self.connected = False
        self.steering_sub={}
        self.drive_sub={}

        # Instantiate the right filter based on launch parameters
        initial_vec = [self.initial_x, self.initial_y, self.initial_theta]
        initial_unc = [0.01, 0.01, 0.01]
        self.mapper = MappingKF(initial_vec,initial_unc)
        self.pose_pub = rospy.Publisher("~pose",PoseStamped)

        # print "Initialising wheel data structure"
        for k in prefix:
            self.steering_sub[k] = message_filters.Subscriber("/vrep/%s/%sSteerEncoder" % (self.name,k), JointState)
            self.drive_sub[k] = message_filters.Subscriber("/vrep/%s/%sDriveEncoder" % (self.name,k), JointState)
            # print "Initialised wheel " + k
        self.ts = message_filters.TimeSynchronizer(self.steering_sub.values()+self.drive_sub.values(), 10)
        self.ts.registerCallback(self.sync_odo_cb)

    def sync_odo_cb(self,*args):
        self.connected = True
        if not self.ready:
            return
        if len(args)!=12:
            rospy.logerr("Invalid number of argument in OdoCallback")
            return
        steering_val = [s.position[0] for s in args[0:6]]
        drive_val = [s.position[0] for s in args[6:12]]
        motors = RoverMotors()
        motors.steering = dict(zip(self.steering_sub.keys(),steering_val))
        motors.drive = dict(zip(self.drive_sub.keys(),drive_val))
        self.odo_cb(args[0].header.stamp,motors)

    def odo_cb(self,timestamp,motors):
        # Get the pose of all drives
        drive_cfg={}
        for k in prefix:
            # try:
                # self.listener.waitForTransform('/%s/ground'%(self.name),
                #         '/%s/%sDrive'%(self.name,k), self.last_cmd, rospy.Duration(1.0))
                ((x,y,z),rot) = self.listener.lookupTransform('/%s/ground'%(self.name),
                        '/%s/%sDrive'%(self.name,k), rospy.Time(0))
                drive_cfg[k] = DriveConfiguration(self.radius[k],x,y,z)
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #    return
        self.mapper.predict(motors, drive_cfg, self.encoder_precision)
        self.mapper.publish(self.target_frame,timestamp)

    def compass_cb(self, value):
        self.mapper.update_compass(value.data,self.compass_precision)

    def ar_cb(self, markers):
        for m in markers.markers:
            if m.id > 32:
                continue
            self.listener.waitForTransform("/%s/ground"%self.name,m.header.frame_id, m.header.stamp, rospy.Duration(1.0))
            m_pose = PointStamped()
            m_pose.header = m.header
            m_pose.point = m.pose.pose.position
            m_pose = self.listener.transformPoint("/%s/ground"%self.name,m_pose)
            Z = vstack([m_pose.point.x,m_pose.point.y])
            if self.ignore_id:
                self.mapper.update_ar(Z,0,self.ar_precision)
            else:
                self.mapper.update_ar(Z,m.id,self.ar_precision)
        self.mapper.publish(self.target_frame,markers.header.stamp)


    def run(self):
        timeout = True
        rate = rospy.Rate(2)
        rospy.loginfo("Waiting for VREP")
        while (not rospy.is_shutdown()) and (not self.connected):
            rate.sleep()
        if rospy.is_shutdown():
            return
        rospy.loginfo("Waiting for initial transforms")
        rospy.sleep(1.0)
        self.radius={}
        for k in prefix:
            try:
                self.listener.waitForTransform('/%s/ground'%(self.name),
                        '/%s/%sDrive'%(self.name,k), rospy.Time(0), rospy.Duration(5.0))
                ((x,y,z),rot) = self.listener.lookupTransform('/%s/ground'%(self.name),
                        '/%s/%sDrive'%(self.name,k), rospy.Time(0))
                self.radius[k] = z
                rospy.loginfo("Got transform for " + k)
            except tf.Exception,e:
                rospy.logerr("TF exception: " + repr(e))
        self.ready = True
        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_cb)
        self.compass_sub = rospy.Subscriber("/vrep/rover/compass", Float32, self.compass_cb)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        rd = RoverMapping("rover") 
        rd.run()
    except rospy.ROSInterruptException:
        pass
