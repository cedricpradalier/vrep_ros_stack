#!/usr/bin/env python
import roslib; roslib.load_manifest('cb_vs')
import rospy
from cb_detector.msg import Checkerboard
from sensor_msgs.msg import Joy,CameraInfo
from geometry_msgs.msg import Twist,Point
from std_msgs.msg import Float32
import numpy
from numpy.linalg import pinv
import math

class CheckerboardVS:
    def __init__(self):
        rospy.init_node("cb_vs");
        self.scale = rospy.get_param("~scale",1.0)
        self.ref_button = rospy.get_param("~ref_button",3)
        self.Z = rospy.get_param("~Z",1.0)

        self.current_cb = None
        self.info = None
        self.ref_cb = None
        self.lastroi = rospy.Time.now()

        self.joy_sub = rospy.Subscriber("~joy",Joy,self.joy_cb)
        self.roi_sub = rospy.Subscriber("~cb",Checkerboard,self.cb_cb)
        self.info_sub = rospy.Subscriber("~info",CameraInfo,self.store_info)
        self.vel_pub = rospy.Publisher("~twistCommand",Twist)
        self.err_pub = rospy.Publisher("~error",Float32)


    def store_info(self,info):
        self.info = info

    def scale_point(self,P):
        return Point(-(P.x - self.info.K[2])/self.info.K[0], (P.y - self.info.K[5])/self.info.K[4], 0.)

    def joy_cb(self,value):
        if value.buttons[self.ref_button]:
            self.ref_cb = self.current_cb
            rospy.loginfo("Recorded reference Checkerboard")

    def cb_cb(self, value):
        self.current_cb = value
        self.current_cb.points = [self.scale_point(P) for P in self.current_cb.points]
        self.lastroi = self.current_cb.header.stamp

    def computeVS(self):
        twist = Twist()
        if not self.ref_cb or not self.current_cb:
            return twist

        if ((self.ref_cb.num_x != self.current_cb.num_x) \
                or (self.ref_cb.num_y != self.current_cb.num_y)):
            return twist

        
        N = self.ref_cb.num_x * self.ref_cb.num_y
        L = numpy.mat(numpy.zeros((N*2,3)))
        E = numpy.mat(numpy.zeros((N*2,1)))
        i = 0
        for (pstar,p) in zip(self.ref_cb.points,self.current_cb.points):
            L[i+0,0] = -1/self.Z; L[i+0,1] = p.x/self.Z; L[i+0,2] = -(1+p.x*p.x); E[i+0,0] = p.x - pstar.x; 
            L[i+1,0] = 0;         L[i+1,1] = p.y/self.Z; L[i+1,2] = -p.x*p.y;     E[i+1,0] = p.y - pstar.y; 
            i += 2
        err = numpy.linalg.norm(E)
        if err > 0.1:
            C = -self.scale * pinv(L) * E
            # print err
            twist.linear.x = C[1,0]
            twist.linear.y = C[0,0]
            twist.angular.z = C[2,0]
        self.err_pub.publish(Float32(err))
        return twist


    def run(self):
        timeout = True
        rate = rospy.Rate(10)
        rospy.loginfo("Waiting for first camera info")
        t = Twist()
        while (not rospy.is_shutdown()) and ((not self.info) or (not self.current_cb)):
            self.vel_pub.publish(t)
            rate.sleep()
        rospy.loginfo("Starting VS control")
        while not rospy.is_shutdown():
            if (rospy.rostime.get_time() - self.lastroi.to_sec()) < 0.5: 
                if timeout:
                    timeout = False
                    rospy.loginfo("Accepting joystick commands")
                twist = self.computeVS()
                self.vel_pub.publish(twist)
            else:
                if not timeout:
                    timeout = True
                    rospy.loginfo("Timeout: ignoring joystick commands")
                self.vel_pub.publish(Twist())
            rate.sleep()


if __name__ == '__main__':
    try:
        rd = CheckerboardVS() 
        rd.run()
    except rospy.ROSInterruptException:
        pass

