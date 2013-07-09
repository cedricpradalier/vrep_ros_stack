import roslib; roslib.load_manifest('ar_mapping')
import rospy
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped
import tf
import threading

import rover_driver
from rover_driver.rover_kinematics import *



class MappingKF(RoverKinematics):
    def __init__(self, initial_pose, initial_uncertainty):
        RoverKinematics.__init__(self)
        self.lock = threading.Lock()
        self.X = mat(vstack(initial_pose))
        self.P = mat(diag(initial_uncertainty))
        self.idx = {}
        self.pose_pub = rospy.Publisher("~pose",PoseStamped)
        self.marker_pub = rospy.Publisher("~landmarks",MarkerArray)

    def getRotation(self, theta):
        R = mat(zeros((2,2)))
        R[0,0] = cos(theta); R[0,1] = -sin(theta)
        R[1,0] = sin(theta); R[1,1] = cos(theta)
        return R

    
    def predict(self, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return (self.X, self.P)
        # print "-"*32
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
        self.motor_state.copy(motor_state)
        
        # Implement Kalman prediction here
        theta = self.X[2,0]
        Rtheta = mat([[cos(theta), -sin(theta), 0], 
                      [sin(theta),  cos(theta), 0],
                      [         0,           0, 1]]);
        DeltaX = iW*S
        # print self.X
        # print DeltaX
        # print Rtheta * DeltaX
        self.X[0:3,0] = self.X[0:3,0] + Rtheta*DeltaX
        Jx = mat([[1, 0, -sin(theta)*DeltaX[0,0]-cos(theta)*DeltaX[1,0]],
                  [0, 1,  cos(theta)*DeltaX[0,0]-sin(theta)*DeltaX[1,0]],
                  [0, 0,                       1                       ]])
        Js = Rtheta*iW
        Qs = mat(diag([encoder_precision]*len(S)))
        P = self.P[0:3,0:3]
        self.P[0:3,0:3] = Jx * P * Jx.T + Js * Qs * Js.T
        self.lock.release()
        return (self.X,self.P)


    def update_ar(self, Z, id, uncertainty):
        # Z is a dictionary of id->vstack([x,y])
        self.lock.acquire()
        # TODO
        print "Update: Z="+str(Z.T)+" X="+str(self.X.T)+" Id="+str(id)
        (n,_) = self.X.shape
        R = mat(diag([uncertainty,uncertainty]))
        theta = self.X[2,0]
        Rtheta = self.getRotation(theta)
        Rmtheta = self.getRotation(-theta)
        # First run the filter on all the known landmarks
        H = mat(zeros((0, n)))
        if id in self.idx.keys():
            l = self.idx[id]
            H = mat(zeros((2,n)))
            H[0:2,0:2] = -Rmtheta
            H[0:2,2] = mat(vstack([-(self.X[l+0,0]-self.X[0,0])*sin(theta) + (self.X[l+1,0]-self.X[1,0])*cos(theta), \
                                   (self.X[l+0,0]-self.X[0,0])*cos(theta) - (self.X[l+1,0]-self.X[1,0])*sin(theta)]))
            H[0:2,l:l+2] = Rmtheta
            Zpred = Rmtheta * (self.X[l:l+2,0] - self.X[0:2,0])
            S = H * self.P * H.T + R
            K = self.P * H.T * inv(S)
            self.X = self.X + K * (Z - Zpred)
            self.P = (mat(eye(n)) - K * H) * self.P
        else:
            self.idx[id] = n
            # print self.X
            # print Rtheta
            # print Z
            # print self.X[0:2,0]
            # print Rtheta*Z
            # print self.X[0:2,0]+Rtheta*Z
            self.X = numpy.concatenate((self.X, self.X[0:2,0]+(Rtheta*Z)))
            Pnew = mat(diag([uncertainty]*(n+2)))
            Pnew[0:n,0:n] = self.P
            self.P = Pnew
        self.lock.release()
        return (self.X,self.P)

    def update_compass(self, Z, uncertainty):
        self.lock.acquire()
        # TODO
        print "Update: S="+str(Z)+" X="+str(self.X.T)
        (n,_) = self.X.shape
        R = mat(diag([uncertainty]))
        Zpred = self.X[2,0]
        H = mat(zeros(self.X.shape)).T
        H[0,2] = 1
        S = H * self.P * H.T + R
        K = self.P * H.T * inv(S)
        # print "P="; print self.P
        # print "H="; print H
        # print "S="; print S
        # print "K="; print K
        # print "I="; print Z-Zpred
        # print "G="; print K*(Z-Zpred)
        I = ((Z-Zpred+math.pi) % (2*math.pi))-math.pi
        self.X = self.X + K * I
        self.P = (mat(eye(n)) - K * H) * self.P
        self.lock.release()
        return (self.X,self.P)


    def publish(self, target_frame, timestamp):
        pose = PoseStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = timestamp
        pose.pose.position.x = self.X[0,0]
        pose.pose.position.y = self.X[1,0]
        pose.pose.position.z = 0.0
        Q = tf.transformations.quaternion_from_euler(0, 0, self.X[2,0])
        pose.pose.orientation.x = Q[0]
        pose.pose.orientation.y = Q[1]
        pose.pose.orientation.z = Q[2]
        pose.pose.orientation.w = Q[3]
        self.pose_pub.publish(pose)
        ma = MarkerArray()
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 5000
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = -0.1
        marker.scale.x = 3*sqrt(self.P[0,0])
        marker.scale.y = 3*sqrt(self.P[1,1]);
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        ma.markers.append(marker)
        for id in self.idx.iterkeys():
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            l = self.idx[id]
            marker.pose.position.x = self.X[l,0]
            marker.pose.position.y = self.X[l+1,0]
            marker.pose.position.z = -0.1
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.x = 3*sqrt(self.P[l,l])
            marker.scale.y = 3*sqrt(self.P[l+1,l+1]);
            marker.scale.z = 0.1;
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
            marker.pose.position.x = self.X[l+0,0]
            marker.pose.position.y = self.X[l+1,0]
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

