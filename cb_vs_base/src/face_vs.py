#!/usr/bin/env python
import roslib; roslib.load_manifest('cb_vs_base')
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


    # store the camera parameters
    def store_info(self,info):
        self.info = info

    # Called whenever the joystick is ready
    def joy_cb(self,value):
        if value.buttons[self.ref_button]:
            self.ref_cb = self.current_cb
            rospy.loginfo("Recorded reference Checkerboard")

    # called whenever a new checkerboard has been detected
    def cb_cb(self, value):
        self.current_cb = value
        self.lastroi = self.current_cb.header.stamp

    def computeVS(self):
        twist = Twist()
        if not self.ref_cb or not self.current_cb:
            return twist

        if ((self.ref_cb.num_x != self.current_cb.num_x) \
                or (self.ref_cb.num_y != self.current_cb.num_y)):
            return twist

        
        # Go through the list of features
        for (pstar,p) in zip(self.ref_cb.points,self.current_cb.points):
            # pstar is a geometry_msgs/Point object in the reference image
            # p is a geometry_msgs/Point object in the current image
            # By construction the matching between p* and p is garanteed.
            do_something_with(pstar,p)

        twist = something()

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

