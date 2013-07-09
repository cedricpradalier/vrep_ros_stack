import roslib; roslib.load_manifest('ar_loc')
import rospy
from numpy import *
from numpy.linalg import pinv, inv
from math import pi, sin, cos
from geometry_msgs.msg import *
import tf
import bisect
import threading

import rover_driver
from rover_driver.rover_kinematics import *

class RoverPF(RoverKinematics):
    def __init__(self, initial_pose, initial_uncertainty):
        RoverKinematics.__init__(self)
        self.initial_uncertainty = initial_uncertainty
        self.lock = threading.Lock()
        self.X = mat(vstack(initial_pose))
        self.N = 500
        self.particles = [self.X + self.drawNoise(initial_uncertainty) for i in range(0,self.N)]
        self.pa_pub = rospy.Publisher("~particles",PoseArray)
        # print self.particles

    def getRotation(self, theta):
        R = mat(zeros((2,2)))
        R[0,0] = cos(theta); R[0,1] = -sin(theta)
        R[1,0] = sin(theta); R[1,1] = cos(theta)
        return R
    
    def getRotationFromWorldToRobot(self):
        return getRotation(-self.X[2,0])

    def drawNoise(self, norm):
        if type(norm)==list:
            return mat(vstack(norm)*(2*random.rand(3,1)-vstack([1,1,1])))
        else:
            return mat(multiply(norm,((2*random.rand(3,1)-vstack([1,1,1])))))

    def applyDisplacement(self,X,DeltaX,Uncertainty):
        coin = random.random()
        theta = X[2,0]
        Rtheta = mat([[cos(theta), -sin(theta), 0], 
                      [sin(theta),  cos(theta), 0],
                      [         0,           0, 1]]);
        if (coin <0.02):
            return X + Rtheta*(DeltaX + self.drawNoise(10*Uncertainty))
        else:
            return X + Rtheta*(DeltaX + self.drawNoise(Uncertainty))


    def predict(self, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return self.X
        # print "-"*32
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
        self.motor_state.copy(motor_state)
        DeltaX = iW*S
        Uncertainty = iW*mat(vstack([encoder_precision] * len(S)))

        self.particles = [self.applyDisplacement(X,DeltaX,Uncertainty) for X in self.particles]
        self.updateMean()
        self.lock.release()

    def evalParticleAR(self,X, Z, L, Uncertainty):
        x = mat(vstack([X[0,0],X[1,0]])) + self.getRotation(X[2,0])*Z
        e = (x-L)/Uncertainty
        return exp(-0.5*e.T*e)

    def evalParticleCompass(self,X, Value, Uncertainty):
        e = (X[2,0]-Value)/Uncertainty
        return exp(-0.5*e*e)

    def update_ar(self, Z, L, Uncertainty):
        self.lock.acquire()
        # TODO
        print "Update: L="+str(L.T)+" X="+str(self.X.T)
        # print self.particles
        alpha=[self.evalParticleAR(x,Z,L,Uncertainty) for x in self.particles]
        s = sum(alpha)
        pdf = [x/s for x in alpha]
        # resample
        cdf = cumsum(pdf)
        samples = random.rand(len(self.particles))
        idx = [bisect.bisect_left(cdf,x) for x in samples]
        self.particles = [self.particles[i] for i in idx]
        
        self.updateMean()
        self.lock.release()

    def update_compass(self, angle, Uncertainty):
        self.lock.acquire()
        # TODO
        # print self.particles
        print "Update: S="+str(angle)+" X="+str(self.X.T)
        alpha=[self.evalParticleCompass(x,angle,Uncertainty) for x in self.particles]
        s = sum(alpha)
        pdf = [x/s for x in alpha]
        # resample
        cdf = cumsum(pdf)
        samples = random.rand(len(self.particles))
        idx = [bisect.bisect_left(cdf,x) for x in samples]
        self.particles = [self.particles[i] for i in idx]
        
        self.updateMean()
        self.lock.release()

    def updateMean(self):
        X = mat(zeros((3,1)))
        for x in self.particles:
            X += x
        self.X = X / len(self.particles)
        
        return self.X

    def publish(self, pose_pub, target_frame, stamp):
        pose = PoseStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = stamp
        pose.pose.position.x = self.X[0,0]
        pose.pose.position.y = self.X[1,0]
        pose.pose.position.z = 0.0
        Q = tf.transformations.quaternion_from_euler(0, 0, self.X[2,0])
        pose.pose.orientation.x = Q[0]
        pose.pose.orientation.y = Q[1]
        pose.pose.orientation.z = Q[2]
        pose.pose.orientation.w = Q[3]
        pose_pub.publish(pose)

        pa = PoseArray()
        pa.header = pose.header
        for p in self.particles:
            po = Pose()
            po.position.x = p[0,0]
            po.position.y = p[1,0]
            q = tf.transformations.quaternion_from_euler(0, 0, p[2,0])
            po.orientation = Quaternion(*q)
            pa.poses.append(po)
        self.pa_pub.publish(pa)

    def broadcast(self,br, target_frame, stamp):
        br.sendTransform((self.X[0,0], self.X[1,0], 0),
                     tf.transformations.quaternion_from_euler(0, 0, self.X[2,0]),
                     stamp, "/%s/ground"%self.name, target_frame)
        

