import roslib; roslib.load_manifest('ar_mapping')
import rospy
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
import tf
import threading

import rover_driver
from rover_driver.rover_kinematics import *

class Landmark:
    def __init__(self, Z, X , R):
        x = X[0,0]
        y = X[1,0]
        theta = X[2,0]
        Rtheta = mat([[cos(theta),-sin(theta)],
            [sin(theta), cos(theta)]]);
        self.L =vstack([x,y]) + dot(Rtheta,Z)
        print X
        print Z
        print Rtheta
        print self.L
        print R
        self.P = R

    def update(self,Z, X, R):
        x = X[0,0]
        y = X[1,0]
        theta = X[2,0]
        Rmtheta = mat([[cos(theta),sin(theta)],
            [-sin(theta), cos(theta)]]);
        H = Rmtheta
        Zpred = Rmtheta * (self.L - vstack([x,y]))
        S = H * self.P * H.T + R
        K = self.P * H.T * inv(S)
        self.L = self.L + K * (Z - Zpred)
        self.P = (mat(eye(2)) - K * H) * self.P
        


class MappingKF:
    def __init__(self):
        self.lock = threading.Lock()
        self.marker_list = {}
        self.marker_pub = rospy.Publisher("~landmarks",MarkerArray)

    def update_ar(self, Z, X, Id, uncertainty):
        self.lock.acquire()
        # TODO
        print "Update: Z="+str(Z.T)+" X="+str(X.T)+" Id="+str(Id)
        R = mat(diag([uncertainty,uncertainty]))
        if Id in self.marker_list.keys():
            # Known landmark, we can run the KF update
            self.marker_list[Id].update(Z,X,R)
        else:
            # New landmark, we need to create it
            self.marker_list[Id] = Landmark(Z,X,R)
            rospy.loginfo("Initialised landmark %d at %s" %
                    (Id,str(self.marker_list[Id].L)))
        self.lock.release()


    def publish(self, target_frame, timestamp):
        ma = MarkerArray()
        for id in self.marker_list:
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0,0]
            marker.pose.position.y = Lkf.L[1,0]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.x = max(3*sqrt(Lkf.P[0,0]),0.05)
            marker.scale.y = max(3*sqrt(Lkf.P[1,1]),0.05)
            marker.scale.z = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime.secs=3.0;
            ma.markers.append(marker)
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = 1000+id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0,0]
            marker.pose.position.y = Lkf.L[1,0]
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.text = str(id)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.lifetime.secs=3.0;
            ma.markers.append(marker)
        self.marker_pub.publish(ma)

