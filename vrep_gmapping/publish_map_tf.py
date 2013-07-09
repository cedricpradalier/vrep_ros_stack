#!/usr/bin/env python
import roslib; roslib.load_manifest('vrep_gmapping')
import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid
import tf

info = None
p = None
q = None
frame = "/world"

def metadata_cb(data):
    global p, q, info
    info = data
    p = data.origin.position
    q = data.origin.orientation
    rospy.loginfo("Received map transform")

def map_cb(data):
    global frame
    frame = data.header.frame_id
    rospy.loginfo("Received map frame")



if __name__ == '__main__':
    try:
        rospy.init_node("map_tf_broadcaster")
        zero = rospy.get_param("~zero",False)
        reference_frame = rospy.get_param("~reference_frame","/world")
        broadcaster = tf.TransformBroadcaster()
        metadata_sub = rospy.Subscriber("/map_metadata",MapMetaData,metadata_cb)
        map_sub = rospy.Subscriber("/map",OccupancyGrid,map_cb)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if p and q:
                if zero:
                    broadcaster.sendTransform((-info.width*info.resolution/2,-info.height*info.resolution/2,0),
                            (0,0,0,1),rospy.Time.now(),frame,reference_frame)
                else:
                    broadcaster.sendTransform((p.x,p.y,p.z),
                            (q.x,q.y,q.z,q.w),rospy.Time.now(),frame,reference_frame)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
