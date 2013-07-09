#!/usr/bin/env ipython
import roslib
roslib.load_manifest('robot_control')

import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from visualization_msgs.msg import Marker
import message_filters
from math import atan2, hypot, pi, cos, sin, pi, fmod, exp
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from robot_control.cfg import GotoXYThetaConfig

def norm_angle(x):
    return ((x+pi)%(2*pi))-pi

class GotoXYTheta:
    def __init__(self):
        rospy.init_node('goto_xy')
        self.reference_frame = rospy.get_param("~reference_frame","/odom")
        self.body_frame = rospy.get_param("~body_frame","/body")
        self.radius = rospy.get_param("~radius",0.1)
        self.k_v = rospy.get_param("~k_v",0.3)
        self.k_alpha = rospy.get_param("~k_alpha",8.0)
        self.k_beta = rospy.get_param("~k_beta",-1.5)
        self.max_speed = rospy.get_param("~max_speed",0.1)
        self.max_rot_speed = rospy.get_param("~max_rot_speed",1.0)
        rospy.loginfo("Starting goto xy (body: '%s', ref: '%s')"%(self.body_frame,self.reference_frame))
        self.listener = tf.TransformListener()
        self.reconfig_srv = Server(GotoXYThetaConfig, self.reconfig_cb)
        rospy.sleep(1.0)
        ((x,y,_),rot) = self.listener.lookupTransform(self.reference_frame, self.body_frame, rospy.Time(0))
        euler = euler_from_quaternion(rot)
        self.goal_x = x
        self.goal_y = y
        self.goal_theta = euler[2]
        print [self.goal_x, self.goal_y, self.goal_theta]

        self.pose_sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.pose_cb)
        self.twist_pub = rospy.Publisher("/vrep/twistCommand",Twist)
        self.marker_pub = rospy.Publisher("~/goal_marker",Marker)
    
    def reconfig_cb(self,config, level):
        self.radius = config["dist_threshold"]
        self.k_v = config["k_v"]
        self.k_alpha = config["k_alpha"]
        self.k_beta = config["k_beta"]
        self.max_speed = config["max_velocity"]
        self.max_rot_speed = config["max_rotational_velocity"]
        # print "Reconfigured"
        return config


    def pose_cb(self, pose):
        pose = self.listener.transformPose(self.reference_frame,pose)
        q = pose.pose.orientation
        qvec = [q.x,q.y,q.z,q.w]
        euler = euler_from_quaternion(qvec)
        self.goal_x = pose.pose.position.x
        self.goal_y = pose.pose.position.y
        self.goal_theta = euler[2]
        print "New goal: %.2f %.2f %.2f" % (self.goal_x, self.goal_y, self.goal_theta)

        marker = Marker()
        marker.header = pose.header
        marker.ns = "goal_marker"
        marker.id = 10
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime.secs=-1.0
        self.marker_pub.publish(marker)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ((x,y,_),rot) = self.listener.lookupTransform(self.reference_frame, self.body_frame, rospy.Time(0))
            euler = euler_from_quaternion(rot)
            theta = euler[2] 
            twist = Twist()
            alpha = norm_angle(atan2(self.goal_y-y, self.goal_x-x) - theta)
            beta = norm_angle(-alpha - theta)
            rho = hypot(y - self.goal_y, x - self.goal_x)
            # print [x - self.goal_x, y-self.goal_y, theta, alpha, rho, dtheta, euler]
            if rho > self.radius:
                twist.linear.x = min(self.max_speed, self.k_v * rho)
                twist.angular.z = max(-self.max_rot_speed, min(self.max_rot_speed, 
                    self.k_alpha * alpha + self.k_beta * beta))
            else:
                twist.linear.x = 0
                twist.angular.z = 0
            # print [twist.linear.x,twist.angular.z]
            self.twist_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        rd = GotoXYTheta() 
        rd.run()
    except rospy.ROSInterruptException:
        pass
