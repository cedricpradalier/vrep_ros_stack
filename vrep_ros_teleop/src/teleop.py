#!/usr/bin/env python
import roslib; roslib.load_manifest('vrep_ros_teleop')
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

last_joy = -1e10
joy_value = None

def joy_cb(value):
    global joy_value
    global last_joy
    last_joy = rospy.rostime.get_time()
    joy_value = value

def talker():
    global joy_value
    global last_joy
    rospy.init_node('vrep_ros_teleop')
    sub = rospy.Subscriber('~joy', Joy, joy_cb)
    pub = rospy.Publisher('~twistCommand', Twist)
    axis_linear = rospy.get_param("~axis_linear",1)
    axis_angular = rospy.get_param("~axis_angular",0)
    scale_linear = rospy.get_param("~scale_linear",5.)
    scale_angular = rospy.get_param("~scale_angular",10.)
    timeout = True
    while not rospy.is_shutdown():
        twist = Twist()
        if (rospy.rostime.get_time() - last_joy) < 0.5: 
            if timeout:
                timeout = False
                rospy.loginfo("Accepting joystick commands")
            twist.linear.x = joy_value.axes[axis_linear] * scale_linear
            twist.angular.z = joy_value.axes[axis_angular] * scale_angular
            if twist.linear.x < 0:
                twist.angular.z = - twist.angular.z
        else:
            if not timeout:
                timeout = True
                rospy.loginfo("Timeout: ignoring joystick commands")
        pub.publish(twist)
        rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
