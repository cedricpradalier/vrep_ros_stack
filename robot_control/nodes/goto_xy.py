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
from robot_control.cfg import GotoXYConfig

def norm_angle(x):
    return ((x+pi)%(2*pi))-pi

class GotoXY:
    def __init__(self):
        rospy.init_node('goto_xy')
        self.reference_frame = rospy.get_param("~reference_frame","/odom")
        self.body_frame = rospy.get_param("~body_frame","/body")
        self.radius = rospy.get_param("~radius",0.1)
        self.k_v = rospy.get_param("~k_v",1.0)
        self.k_alpha = rospy.get_param("~k_alpha",0.1)
        self.max_speed = rospy.get_param("~max_speed",0.1)
        self.max_rot_speed = rospy.get_param("~max_rot_speed",1.0)
        rospy.loginfo("Starting goto xy (body: '%s', ref: '%s')"%(self.body_frame,self.reference_frame))
        self.listener = tf.TransformListener()
        self.reconfig_srv = Server(GotoXYConfig, self.reconfig_cb)
        rospy.sleep(1.0)
        ((x,y,_),_) = self.listener.lookupTransform(self.reference_frame, self.body_frame, rospy.Time(0))
        self.goal_x = x
        self.goal_y = y
        print [self.goal_x, self.goal_y]

        self.pose_sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.pose_cb)
        self.twist_pub = rospy.Publisher("~twistCommand",Twist)
        self.marker_pub = rospy.Publisher("~goal_marker",Marker)
    
    def reconfig_cb(self,config, level):
        self.radius = config["dist_threshold"]
        self.k_v = config["k_v"]
        self.k_alpha = config["k_alpha"]
        self.max_speed = config["max_velocity"]
        self.max_rot_speed = config["max_rotational_velocity"]
        # print "Reconfigured"
        return config


    def pose_cb(self, pose):
        m_pose = PointStamped()
        m_pose.header = pose.header
        m_pose.point = pose.pose.position
        m_pose = self.listener.transformPoint(self.reference_frame,m_pose)
        self.goal_x = m_pose.point.x
        self.goal_y = m_pose.point.y
        print "New goal: %.2f %.2f" % (self.goal_x, self.goal_y)

        marker = Marker()
        marker.header = pose.header
        marker.ns = "goal_marker"
        marker.id = 10
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.5;
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
            alpha = atan2(self.goal_y-y, self.goal_x-x)
            rho = hypot(y - self.goal_y, x - self.goal_x)
            dtheta = norm_angle(alpha - theta)
            # print [x - self.goal_x, y-self.goal_y, theta, alpha, rho, dtheta, euler]
            if rho > self.radius:
                twist.linear.x = min(self.max_speed, self.k_v * rho * exp(-(3*dtheta)**2))
                twist.angular.z = max(-self.max_rot_speed, min(self.max_rot_speed, self.k_alpha * dtheta))
            else:
                twist.linear.x = 0
                twist.angular.z = 0
            # print [twist.linear.x,twist.angular.z]
            # twist = Twist()
            # twist.linear.x = 0
            # twist.angular.z = 0.5
            self.twist_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        rd = GotoXY() 
        rd.run()
    except rospy.ROSInterruptException:
        pass
