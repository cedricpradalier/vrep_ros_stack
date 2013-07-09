#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

tc.WaitForAuto()
try:
    tc.GoTo(goal_x=-1.0,goal_y=0.0)
    tc.Wait(duration=1.0)
    tc.GoTo(goal_x=0.0,goal_y=1.0)
    tc.Wait(duration=1.0)
    tc.GoTo(goal_x=1.0,goal_y=0.0)
    tc.Wait(duration=1.0)
    tc.GoTo(goal_x=0.0,goal_y=-1.0)

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")
