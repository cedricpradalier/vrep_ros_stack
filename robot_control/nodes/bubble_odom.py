#!/usr/bin/env python
import roslib
roslib.load_manifest('robot_control')

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import message_filters
from math import atan2, hypot, pi, cos, sin, fmod

def norm_angle(x):
    return fmod(x+pi,2*pi)-pi


class BubbleOdom:
    def __init__(self):
        rospy.init_node('bubble_odom')
        rospy.loginfo("Starting bubble rob odometry")
        self.last_cmd = rospy.Time.now()
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.drive_sub={}
        self.ready = False
        self.connected = False
        self.left_prev = None
        self.right_prev = None
        self.pose = {"x":0, "y":0, "theta":0}
        rospy.sleep(0.5)
        ((_,yl,z),_) = self.listener.lookupTransform('/body', '/left_wheel', rospy.Time(0))
        ((_,yr,_),_) = self.listener.lookupTransform('/body', '/right_wheel', rospy.Time(0))
        self.e = abs(yl - yr)
        self.wheel_radius = z
        print "Inter wheel and radius : %f %f" % (self.e,self.wheel_radius)


        self.drive_sub["Left"] = message_filters.Subscriber("/vrep/leftWheelEncoder", JointState)
        self.drive_sub["Right"] = message_filters.Subscriber("/vrep/rightWheelEncoder", JointState)
        self.ts = message_filters.TimeSynchronizer([self.drive_sub["Left"],self.drive_sub["Right"]], 10)
        self.ts.registerCallback(self.sync_odo_cb)

    def sync_odo_cb(self,left,right):
        if not self.connected:
            self.left_prev = left
            self.right_prev = right
            self.connected = True
            return
        dleft = norm_angle(left.position[0] - self.left_prev.position[0])
        dright = norm_angle(right.position[0] - self.right_prev.position[0])
        if dleft > pi:
            dleft -= 2*pi
        elif dleft < -pi:
            dleft += 2*pi
        if dright > pi:
            dright -= 2*pi
        elif dright < -pi:
            dright += 2*pi
        dleft *= self.wheel_radius
        dright *= self.wheel_radius
        dx = (dleft + dright)/2.0
        dy = 0
        dtheta = (dright - dleft)/self.e
        # print [dleft, dright, dx, dy ,dtheta]
        self.pose["x"] += dx * cos(self.pose["theta"]) - dy * sin(self.pose["theta"])
        self.pose["y"] += dx * sin(self.pose["theta"]) + dy * cos(self.pose["theta"])
        self.pose["theta"] += dtheta
        self.broadcaster.sendTransform((self.pose["x"], self.pose["y"], 0),
                     tf.transformations.quaternion_from_euler(0, 0, self.pose["theta"]),
                     left.header.stamp, "/body", "/odom")
        self.left_prev = left
        self.right_prev = right




    def run(self):
        rospy.spin()


if __name__=="__main__":
    demo = BubbleOdom()
    demo.run()
