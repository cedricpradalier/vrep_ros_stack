#!/usr/bin/env python
import roslib; roslib.load_manifest('sift_detect')
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from topic_tools.srv import MuxSelect
from geometry_msgs.msg import Twist

mux_proxy = None
joystick_button = 0
joystick_topic = "/joy/topic"
servo_button = 1
servo_topic = "/servo/topic"
selected = ""
last_joy = -1e10

def joy_cb(value):
    global mux_proxy, joystick_topic, joystick_button
    global servo_topic, servo_button, selected, last_joy
    last_joy = rospy.rostime.get_time()
    if not mux_proxy:
        return
    try:
        if value.buttons[joystick_button] and (selected.data != joystick_topic):
            rospy.loginfo("Selecting joystick")
            mux_proxy(joystick_topic)
        elif value.buttons[servo_button] and (selected.data != servo_topic):
            rospy.loginfo("Selecting servo")
            mux_proxy(servo_topic)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def selected_cb(value):
    global selected
    selected = value


def init():
    global mux_proxy, joystick_topic, joystick_button
    global servo_topic, servo_button, selected, last_joy
    rospy.init_node('vrep_ros_teleop_mux')
    rospy.wait_for_service('/mux/select')
    mux_proxy = rospy.ServiceProxy('/mux/select', MuxSelect)
    
    sub = rospy.Subscriber('~joy', Joy, joy_cb)
    sub = rospy.Subscriber('/mux/selected', String, selected_cb)
    joystick_button = rospy.get_param("~joystick_button",joystick_button)
    joystick_topic = rospy.get_param("~joystick_topic",joystick_topic)
    servo_button = rospy.get_param("~servo_button",servo_button)
    servo_topic = rospy.get_param("~servo_topic",servo_topic)
    timeout = False
    while not rospy.is_shutdown():
        if (rospy.rostime.get_time() - last_joy) < 0.5: 
            if timeout:
                timeout = False
        else:
            if not timeout:
                rospy.loginfo("Timeout: reverting to joystick command")
                timeout = True
                mux_proxy(joystick_topic)
        rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
