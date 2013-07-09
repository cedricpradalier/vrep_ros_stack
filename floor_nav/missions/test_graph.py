#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
import igraph as ig
from math import *
from task_manager_lib.TaskClient import *
from roslib.packages import get_pkg_dir

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

g = ig.Graph.Read_Picklez(get_pkg_dir('floor_graph')+"/graph.picklez")

tc.WaitForAuto()
try:
    tc.Wait(duration=1.0)
    # Hand-made hamiltonian path
    node_seq = [0, 2, 1, 4, 3, 5, 6, 11, 7, 10, 9, 8]
    for node in node_seq:
        tc.GoTo(goal_x=g.vs[node]["x"],goal_y=g.vs[node]["y"])
    tc.Wait(duration=1.0)

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")
