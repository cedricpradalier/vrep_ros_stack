#!/usr/bin/env python
import roslib; roslib.load_manifest('vrep_ros_teleop')
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from topic_tools.srv import MuxSelect

mux_proxy = None
joystick_button = 0
joystick_topic = "/joy/topic"
auto_button = 1
auto_topic = "/auto/topic"
selected = ""
last_joy = -1e10

def joy_cb(value):
    global mux_proxy, joystick_topic, joystick_button
    global auto_topic, auto_button, selected, last_joy
    last_joy = rospy.rostime.get_time()
    if not mux_proxy:
        return
    try:
        if value.buttons[joystick_button] and (selected.data != joystick_topic):
            rospy.loginfo("Selecting joystick")
            mux_proxy(joystick_topic)
        elif value.buttons[auto_button] and (selected.data != auto_topic):
            rospy.loginfo("Selecting auto")
            mux_proxy(auto_topic)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def selected_cb(value):
    global selected
    selected = value



def init():
    global mux_proxy, joystick_topic, joystick_button
    global auto_topic, auto_button, selected, last_joy
    rospy.init_node('vrep_ros_teleop_mux')
    rospy.wait_for_service('/mux/select')
    mux_proxy = rospy.ServiceProxy('/mux/select', MuxSelect)
    
    sub = rospy.Subscriber('~joy', Joy, joy_cb)
    sub = rospy.Subscriber('/mux/selected', String, selected_cb)
    joystick_button = rospy.get_param("~joystick_button",joystick_button)
    joystick_topic = rospy.get_param("~joystick_topic",joystick_topic)
    auto_button = rospy.get_param("~auto_button",auto_button)
    auto_topic = rospy.get_param("~auto_topic",auto_topic)
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
