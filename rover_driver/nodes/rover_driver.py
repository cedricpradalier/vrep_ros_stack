#!/usr/bin/env python
import roslib; roslib.load_manifest('rover_driver')
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist,Pose
from math import atan2, hypot, pi, cos, sin
import tf
import message_filters
import numpy
from numpy.linalg import pinv

from rover_kinematics import *


class RoverDriver:
    def __init__(self,name):
        self.name = name
        rospy.init_node('rover_driver')
        self.name = rospy.get_param("~rover_name",self.name)
        self.skidsteer = rospy.get_param("~skidsteer",False)
        self.check_timeout = rospy.get_param("~check_timeout",True)
        rospy.loginfo("Starting rover driver for rover '%s'" % self.name)
        self.last_cmd = rospy.Time.now()
        self.listener = tf.TransformListener()
        self.steering_pub={}
        self.drive_pub={}
        self.ready = False

        self.kinematics = RoverKinematics()

        self.twist_sub = rospy.Subscriber('~twistCommand', Twist, self.twist_cb)
        # print "Initialising wheel data structure"
        for k in prefix:
            self.steering_pub[k] = rospy.Publisher("/vrep/%s/%sSteerCommand" % (self.name,k), Float64)
            self.drive_pub[k] = rospy.Publisher("/vrep/%s/%sDriveCommand" % (self.name,k), Float64)


    def twist_cb(self,twist):
        if not self.ready:
            return
        # print "Got twist: " + str(twist)
        self.last_cmd = rospy.Time.now()
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
        # Now compute for each drive, its rotation speed and steering angle
        motors = self.kinematics.twist_to_motors(twist,drive_cfg,self.skidsteer)
        self.publish(motors)

    def publish(self, motor):
        for k in prefix:
            self.drive_pub[k].publish(Float64(motor.drive[k]))
            self.steering_pub[k].publish(Float64(motor.steering[k]))
            

    def run(self):
        timeout = True
        rate = rospy.Rate(10)
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
        while not rospy.is_shutdown():
            if self.check_timeout:
                if (rospy.rostime.get_time() - self.last_cmd.to_sec()) < 0.5: 
                    if timeout:
                        timeout = False
                        rospy.loginfo("Accepting joystick commands")
                else:
                    if not timeout:
                        timeout = True
                        rospy.loginfo("Timeout: ignoring joystick commands")
                    motors = RoverMotors()
                    self.publish(motors)
            rate.sleep()


if __name__ == '__main__':
    try:
        rd = RoverDriver("rover") 
        rd.run()
    except rospy.ROSInterruptException:
        pass
