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
        self.name = rospy.get_param("~rover_name",self.name)
        self.skidsteer = rospy.get_param("~skidsteer",False)
        rospy.loginfo("Starting rover driver for rover '%s'" % self.name)
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
        self.ready = False
        self.connected = False
        # Necessary due to a bug in the tf listener

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
        # print "-"*32
        # then compute odometry using least square
        W = numpy.asmatrix(numpy.zeros((12,3)))
        S = numpy.asmatrix(numpy.zeros((12,1)))
        for i in range(len(prefix)):
            k = prefix[i]
            # compute differentials
            ds = (motors.drive[k] - self.motor_state.drive[k]) % (2*pi)
            if ds>pi:
                ds -= 2*pi
            ds *= self.radius[k]
            dx = ds*cos((motors.steering[k]+self.motor_state.steering[k])/2)
            dy = ds*sin((motors.steering[k]+self.motor_state.steering[k])/2)
            # print "%s: S %+.2f D %+.2f %+.2f" % (k,ds,dx,dy)
            ((x,y,z),rot) = self.listener.lookupTransform('/%s/ground'%(self.name),
                    '/%s/%sDrive'%(self.name,k), rospy.Time(0))
            # print "%s: %.2f %.2f %.2f" % (k,x,y,z)
            W[2*i+0,0] = 1; W[2*i+0,1] = 0; W[2*i+0,2] = -y; S[2*i+0,0] = dx
            W[2*i+1,0] = 0; W[2*i+1,1] = 1; W[2*i+1,2] = +x; S[2*i+1,0] = dy
        dX = pinv(W) * S 
        self.X[0,0] += dX[0,0]*cos(self.X[2,0]) - dX[1,0]*sin(self.X[2,0])
        self.X[1,0] += dX[0,0]*sin(self.X[2,0]) + dX[1,0]*cos(self.X[2,0])
        self.X[2,0] += dX[2,0]
        self.broadcaster.sendTransform((self.X[0,0], self.X[1,0], 0),
                     tf.transformations.quaternion_from_euler(0, 0, self.X[2,0]),
                     timestamp, "/%s/ground"%self.name, "/odom")
        # print self.X.T
        # finally store the value of the motor state
        self.motor_state.copy(motors)

    def twist_cb(self,twist):
        if not self.ready:
            return
        # print "Got twist: " + str(twist)
        self.last_cmd = rospy.Time.now()
        # Get the pose of all drives
        drive_pose={}
        for k in prefix:
            # try:
                # self.listener.waitForTransform('/%s/ground'%(self.name),
                #         '/%s/%sDrive'%(self.name,k), self.last_cmd, rospy.Duration(1.0))
                ((x,y,z),rot) = self.listener.lookupTransform('/%s/ground'%(self.name),
                        '/%s/%sDrive'%(self.name,k), rospy.Time(0))
                drive_pose[k] = {"x":x,"y":y,"z":z}
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #    return
        # Now compute for each drive, its rotation speed and steering angle
        motors = RoverMotors()
        # print "-"*32
        if self.skidsteer:
            for k in drive_pose.keys():
                # First compute the speed from each wheel 
                #     V_wheel = V_body + Omega x R
                vw_x = twist.linear.x - 2*twist.angular.z*drive_pose[k]["y"]
                vw_y = 0
                motors.steering[k] = 0
                motors.drive[k] = vw_x / self.radius[k]
        else:
            for k in drive_pose.keys():
                # First compute the speed from each wheel 
                #     V_wheel = V_body + Omega x R
                vw_x = twist.linear.x - twist.angular.z*drive_pose[k]["y"]
                vw_y = twist.linear.y + twist.angular.z*drive_pose[k]["x"]
                motors.steering[k] = atan2(vw_y,vw_x); 
                motors.drive[k] = hypot(vw_y,vw_x) / self.radius[k]
                if motors.steering[k] > pi/2:
                    motors.steering[k] -= pi
                    motors.drive[k] = -motors.drive[k]
                if motors.steering[k] <-pi/2:
                    motors.steering[k] += pi
                    motors.drive[k] = -motors.drive[k]
                # print "%s: T %.2f %.2f %.2f V %.2f %.2f S %.2f D %.2f" % \
                #     (k,drive_pose[k]["x"],drive_pose[k]["y"],drive_pose[k]["z"],\
                #     vw_x,vw_y,motors.steering[k]*180./pi,motors.drive[k])
        self.publish(motors)

    def publish(self, motor):
        for k in prefix:
            self.drive_pub[k].publish(Float64(-motor.drive[k]))
            self.steering_pub[k].publish(Float64(motor.steering[k]))
            

    def run(self):
        timeout = True
        rate = rospy.Rate(10)
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
