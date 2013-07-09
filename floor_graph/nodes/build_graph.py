#!/usr/bin/env python
import roslib; roslib.load_manifest('floor_graph')
import rospy
import tf
import igraph as ig

rospy.init_node('build_graph')
listener = tf.TransformListener()
rospy.sleep(0.5);


g = ig.Graph();
g.add_vertices(12)
g.add_edges([(0,i) for i in [1,2,6,9,11]])
g.add_edges([(1,i) for i in [2,3,4,9]])
g.add_edges([(2,i) for i in [3,6]])
g.add_edges([(3,i) for i in [4,5]])
g.add_edges([(5,i) for i in [6]])
g.add_edges([(6,i) for i in [7,11]])
g.add_edges([(7,i) for i in [10,11]])
g.add_edges([(8,i) for i in [9]])
g.add_edges([(9,i) for i in [10]])
# 11, 10 and 4 do not connect to a higher-index vertex

vx=[0.0] * len(g.vs)
vy=[0.0] * len(g.vs)
label=[""] * len(g.vs)
for i,v in enumerate(g.vs):
    ((x,y,z),rot) = listener.lookupTransform('/world',"/Node%d"%v.index, rospy.Time(0))
    vx[i] = x
    vy[i] = y
    label[i] = "Node%d" % v.index
g.vs["x"] = vx
g.vs["y"] = vy
g.vs["label"] = label

layout = g.layout("auto")
ig.plot(g,'graph.png', layout = layout, label_dist=[100.]*len(g.vs))
g.write_picklez("graph.picklez")
print "Saved graph"



