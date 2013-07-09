import roslib; roslib.load_manifest('ar_loc')
import rospy
from numpy import *
from numpy.linalg import pinv, inv
from math import pi, sin, cos
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf
import threading

import rover_driver
from rover_driver.rover_kinematics import *

class RoverKF(RoverKinematics):
    def __init__(self, initial_pose, initial_uncertainty):
        RoverKinematics.__init__(self)
        self.lock = threading.Lock()
        self.X = mat(vstack(initial_pose))
        self.P = mat(diag(initial_uncertainty))
        self.ellipse_pub = rospy.Publisher("~ellipse",Marker)
        self.pose_with_cov_pub = rospy.Publisher("~pose_with_covariance",PoseWithCovarianceStamped)

    def getRotation(self, theta):
        R = mat(zeros((2,2)))
        R[0,0] = cos(theta); R[0,1] = -sin(theta)
        R[1,0] = sin(theta); R[1,1] = cos(theta)
        return R
    
    def getRotationFromWorldToRobot(self):
        return getRotation(-self.X[2,0])

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
        self.X = self.X + Rtheta*DeltaX
        Jx = mat([[1, 0, -sin(theta)*DeltaX[0,0]-cos(theta)*DeltaX[1,0]],
                  [0, 1,  cos(theta)*DeltaX[0,0]-sin(theta)*DeltaX[1,0]],
                  [0, 0,                       1                       ]])
        Js = Rtheta*iW
        Qs = mat(diag([encoder_precision]*len(S)))
        self.P = Jx * self.P * Jx.T + Js * Qs * Js.T
        self.lock.release()
        return (self.X,self.P)

    def update_ar(self, Z, L, uncertainty):
        self.lock.acquire()
        # TODO
        print "Update: L="+str(L.T)+" X="+str(self.X.T)
        R = mat(diag([uncertainty,uncertainty]))
        theta = self.X[2,0]
        Rmtheta = self.getRotation(-theta)
        Zpred = Rmtheta*mat([L[0,0]-self.X[0,0], L[1,0]-self.X[1,0]]).T
        # print "      : Z=" + str(Z.T) + " Zpred="+str(Zpred.T)
        H = mat([[-cos(theta), -sin(theta), -(L[0,0]-self.X[0,0])*sin(theta) + (L[1,0]-self.X[1,0])*cos(theta)],
                 [ sin(theta), -cos(theta), -(L[0,0]-self.X[0,0])*cos(theta) - (L[1,0]-self.X[1,0])*sin(theta)]])
        S = H * self.P * H.T + R
        K = self.P * H.T * inv(S)
        # print "P="; print self.P
        # print "H="; print H
        # print "S="; print S
        # print "K="; print K
        # print "I="; print Z-Zpred
        # print "G="; print K*(Z-Zpred)
        self.X = self.X + K * (Z - Zpred)
        self.P = (mat(eye(3)) - K * H) * self.P
        self.lock.release()
        return (self.X,self.P)

    def update_compass(self, Z, uncertainty):
        self.lock.acquire()
        # TODO
        print "Update: S="+str(Z)+" X="+str(self.X.T)
        R = mat(diag([uncertainty]))
        Zpred = self.X[2,0]
        H = mat(hstack([0,0,1]))
        S = H * self.P * H.T + R
        K = self.P * H.T * inv(S)
        # print "P="; print self.P
        # print "H="; print H
        # print "S="; print S
        # print "K="; print K
        # print "I="; print Z-Zpred
        # print "G="; print K*(Z-Zpred)
        self.X = self.X + K * (Z - Zpred)
        self.P = (mat(eye(3)) - K * H) * self.P
        self.lock.release()
        return (self.X,self.P)

    def publish(self, pose_pub, target_frame, stamp):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = stamp
        pose.pose.pose.position.x = self.X[0,0]
        pose.pose.pose.position.y = self.X[1,0]
        pose.pose.pose.position.z = 0.0
        Q = tf.transformations.quaternion_from_euler(0, 0, self.X[2,0])
        pose.pose.pose.orientation.x = Q[0]
        pose.pose.pose.orientation.y = Q[1]
        pose.pose.pose.orientation.z = Q[2]
        pose.pose.pose.orientation.w = Q[3]
        psub = PoseStamped()
        psub.header = pose.header
        psub.pose = pose.pose.pose
        pose_pub.publish(psub)
        C = [0]*36
        C[ 0] = self.P[0,0]; C[ 1] = self.P[0,1]; C[ 5] = self.P[0,2]
        C[ 6] = self.P[1,0]; C[ 7] = self.P[1,1]; C[11] = self.P[1,2]
        C[30] = self.P[2,0]; C[31] = self.P[2,1]; C[35] = self.P[2,2]
        pose.pose.covariance = C
        self.pose_with_cov_pub.publish(pose)
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose.pose
        marker.scale.x = 3*sqrt(self.P[0,0])
        marker.scale.y = 3*sqrt(self.P[1,1]);
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        self.ellipse_pub.publish(marker)

    def broadcast(self,br, target_frame, stamp):
        br.sendTransform((self.X[0,0], self.X[1,0], 0),
                     tf.transformations.quaternion_from_euler(0, 0, self.X[2,0]),
                     stamp, "/%s/ground"%self.name, target_frame)
        

