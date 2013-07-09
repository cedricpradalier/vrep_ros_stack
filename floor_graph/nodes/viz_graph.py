#!/usr/bin/env python
import roslib; roslib.load_manifest('floor_graph')
import rospy
import tf
import igraph as ig
from visualization_msgs.msg import MarkerArray,Marker
from geometry_msgs.msg import Point
from roslib.packages import get_pkg_dir

rospy.init_node('build_graph')
listener = tf.TransformListener()
mpub = rospy.Publisher("/graph/viz",MarkerArray,latch=True)
rospy.sleep(1.0);

ma = MarkerArray()
now = rospy.Time.now()

g = ig.Graph.Read_Picklez(get_pkg_dir('floor_graph')+"/graph.picklez")
id = 0
for v in g.vs:
    marker = Marker()
    marker.header.stamp = now
    marker.header.frame_id = "/world"
    marker.ns = "graph_nodes"
    marker.id = id; id += 1
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = v["x"]
    marker.pose.position.y = v["y"]
    marker.pose.position.z = -0.05
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    ma.markers.append(marker)
for e in g.es:
    v0 = g.vs[e.source]
    v1 = g.vs[e.target]
    marker = Marker()
    marker.header.stamp = now
    marker.header.frame_id = "/world"
    marker.ns = "graph_edges"
    marker.id = id; id += 1
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = -0.05
    marker.scale.x = 0.05
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.points = [Point(v0["x"],v0["y"],0),Point(v1["x"],v1["y"],0)];
    ma.markers.append(marker)

while not rospy.is_shutdown():
    mpub.publish(ma)
    rospy.sleep(1.0)



