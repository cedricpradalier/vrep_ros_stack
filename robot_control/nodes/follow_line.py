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
from robot_control.cfg import FollowLineConfig

def norm_angle(x):
    return ((x+pi)%(2*pi))-pi

def sinc(x):
    if abs(x)>1e-5:
        return sin(x)/x
    else:
        return 1.

def sat(x, mx):
    if x > mx:
        return mx
    if x < -mx:
        return -mx
    return x

class FollowLine:
    def __init__(self):
        rospy.init_node('goto_xy')
        self.reference_frame = rospy.get_param("~reference_frame","/odom")
        self.body_frame = rospy.get_param("~body_frame","/body")
        self.dist_threshold = rospy.get_param("~dist_threshold",0.1)
        self.velocity = rospy.get_param("~velocity",1.0)
        self.k_y = rospy.get_param("~k_y",0.1)
        self.k_theta = rospy.get_param("~k_theta",0.1)
        self.max_rot_speed = rospy.get_param("~max_angular_velocity",1.0)
        self.max_y_error = rospy.get_param("~max_y_error",2.0)
        rospy.loginfo("Starting goto xy (body: '%s', ref: '%s')"%(self.body_frame,self.reference_frame))
        self.listener = tf.TransformListener()
        self.reconfig_srv = Server(FollowLineConfig, self.reconfig_cb)
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
        self.dist_threshold = config["dist_threshold"]
        self.velocity = config["velocity"]
        self.k_y = config["k_y"]
        self.k_theta = config["k_theta"]
        self.max_y_error = config["max_y_error"]
        self.max_rot_speed = config["max_angular_velocity"]
        # print "Reconfigured"
        return config

    def compute_error(self):
        ((x,y,_),rot) = self.listener.lookupTransform(self.reference_frame, self.body_frame, rospy.Time(0))
        euler = euler_from_quaternion(rot)
        theta = euler[2] 
        u = cos(self.goal_theta)
        v = sin(self.goal_theta)
        dx = self.goal_x - x
        dy = self.goal_y - y
        ex = dx*u + dy*v
        ey = -v*dx + u*dy
        etheta = norm_angle(self.goal_theta - theta)
        return (ex,ey,etheta)


    def pose_cb(self, pose):
        pose = self.listener.transformPose(self.reference_frame,pose)
        q = pose.pose.orientation
        qvec = [q.x,q.y,q.z,q.w]
        euler = euler_from_quaternion(qvec)
        self.goal_x = pose.pose.position.x
        self.goal_y = pose.pose.position.y
        self.goal_theta = euler[2]
        (ex,ey,etheta) = self.compute_error()
        if ex < -self.dist_threshold:
            self.goal_theta = norm_angle(etheta + pi)
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
            (ex, ey, etheta) = self.compute_error()
            # print [ex,ey,etheta]
            ey = sat(ey, self.max_y_error)
            twist = Twist()
            # print [x - self.goal_x, y-self.goal_y, theta, alpha, rho, dtheta, euler]
            if ex > self.dist_threshold:
                twist.linear.x = self.velocity
                if abs(etheta)>pi/3:
                    twist.linear.x *= max((2*pi/3 - abs(etheta))/(pi/3),0) 
                twist.angular.z = sat(self.k_theta * etheta + self.k_y * sinc(etheta) * self.velocity * ey,
                    self.max_rot_speed)
            else:
                twist.linear.x = 0
                twist.angular.z = 0
            # print [twist.linear.x,twist.angular.z]
            self.twist_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        rd = FollowLine() 
        rd.run()
    except rospy.ROSInterruptException:
        pass
