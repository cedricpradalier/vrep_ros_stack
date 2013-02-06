#!/usr/bin/env python
import roslib; roslib.load_manifest('rover_driver_base')
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist,Pose
from math import atan2, hypot, pi, cos, sin
import tf
import message_filters
import numpy
from numpy.linalg import pinv


# Wheel indices: [Front, Center, Rear] x [Left,Right]
prefix=["FL","FR","CL","CR","RL","RR"]

class RoverMotors:
    def __init__(self):
        self.steering={}
        self.drive={}
        for k in prefix:
            self.steering[k]=0.0
            self.drive[k]=0.0
    def copy(self,value):
        for k in prefix:
            self.steering[k]=value.steering[k]
            self.drive[k]=value.drive[k]


class RoverDriver:
    def __init__(self,name):
        self.name = name
        rospy.init_node('rover_driver')
        self.last_cmd = rospy.Time.now()
        self.motor_state = RoverMotors()
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.steering_sub={}
        self.drive_sub={}
        self.steering_pub={}
        self.drive_pub={}
        self.X = numpy.asmatrix(numpy.zeros((3,1)))
        self.first_run = True
        self.connected = False
        self.ready = False
        # Necessary due to a bug in the tf listener
        rospy.sleep(1.0)

        self.twist_sub = rospy.Subscriber('~twistCommand', Twist, self.twist_cb)
        # print "Initialising wheel data structure"
        for k in prefix:
            self.steering_sub[k] = message_filters.Subscriber("/vrep/%s/%sSteerEncoder" % (self.name,k), JointState)
            self.drive_sub[k] = message_filters.Subscriber("/vrep/%s/%sDriveEncoder" % (self.name,k), JointState)
            self.steering_pub[k] = rospy.Publisher("/vrep/%s/%sSteerCommand" % (self.name,k), Float64)
            self.drive_pub[k] = rospy.Publisher("/vrep/%s/%sDriveCommand" % (self.name,k), Float64)
            # print "Initialised wheel " + k
        self.ts = message_filters.TimeSynchronizer(self.steering_sub.values()+self.drive_sub.values(), 10)
        self.ts.registerCallback(self.sync_odo_cb)

        self.radius={}
        for k in prefix:
            ((x,y,z),rot) = self.listener.lookupTransform('/%s/ground'%(self.name),
                    '/%s/%sDrive'%(self.name,k), rospy.Time(0))
            self.radius[k] = z

    def sync_odo_cb(self,*args):
        self.connected = True
        if not self.ready:
            return
        if len(args)!=12:
            rospy.logerr("Invalid number of argument in OdoCallback")
            return
        steering_val = [s.position[0] for s in args[0:6]]
        drive_val = [-s.position[0] for s in args[6:12]]
        motors = RoverMotors()
        motors.steering = dict(zip(self.steering_sub.keys(),steering_val))
        motors.drive = dict(zip(self.drive_sub.keys(),drive_val))
        if self.first_run:
            self.motor_state.copy(motors)
            self.first_run = False
        else:
            self.odo_cb(args[0].header.stamp,motors)

    def odo_cb(self,timestamp,motors):
        print "-"*16 + " Measurements " + "-"*16
        for k in prefix:
            print "%s: Steering %+5.2f rad Drive %+5.2f rad" % (k,motors.steering[k],motors.drive[k]) 
        # Use the following information to compute the rover displacement
        # - self.motor_state: the previous value of the joint state
        # - self.radius: the wheel radius
        # Additonally, you need to get the transform from "/name/ground" to
        # "/name/drive" for each wheel to know where the wheel is w.r.t the 
        # rover reference point on the ground (/name/ground)

        # Your code here

        # Finally use the computed displacement to update the current position
        # X in 2D (To be modified)
        self.X[0,0] += 0.0
        self.X[1,0] += 0.0
        self.X[2,0] += 0.0
        # Broadcast the new displacement as a TF
        self.broadcaster.sendTransform((self.X[0,0], self.X[1,0], 0),
                     tf.transformations.quaternion_from_euler(0, 0, self.X[2,0]),
                     timestamp, "/%s/ground"%self.name, "/%s/odom"%self.name)
        # finally store the value of the motor state
        self.motor_state.copy(motors)

    def twist_cb(self,twist):
        if not self.ready:
            return
        self.last_cmd = rospy.Time.now()
        motors = RoverMotors()
        # Use the twist to compute the appropriate velocity for each motor
        # Use the transform_listener to get the pose of all drives w.r.t the
        # rover reference point on the ground /name/ground
        print "-"*16 + " Controls " + "-"*16
        for k in prefix:
            # compute the speed for each wheel and affect it as follows:
            motors.steering[k] = 0.0
            motors.drive[k] = 0.0
            print "%s: Steering %+5.2f rad Drive %+5.2f rad" % (k,motors.steering[k],motors.drive[k]) 
        # Finally publish it to all the motors
        self.publish(motors)

    def publish(self, motor):
        for k in prefix:
            self.drive_pub[k].publish(Float64(-motor.drive[k]))
            self.steering_pub[k].publish(Float64(motor.steering[k]))
            

    def run(self):
        timeout = True
        rate = rospy.Rate(10)
        rospy.loginfo("Waiting for VREP")
        while not rospy.is_shutdown() and not self.connected:
            rate.sleep()
        rospy.loginfo("Waiting for initial transforms")
        for k in prefix:
            try:
                self.listener.waitForTransform('/%s/ground'%(self.name),
                        '/%s/%sDrive'%(self.name,k), rospy.Time(0), rospy.Duration(5.0))
                rospy.log_info("Got transform for " + k)
            except tf.Exception,e:
                rospy.logerr("TF exception: " + repr(e))
        self.ready = True
        while not rospy.is_shutdown():
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
