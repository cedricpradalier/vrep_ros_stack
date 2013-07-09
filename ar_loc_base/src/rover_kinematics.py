#!/usr/bin/env python
import roslib; roslib.load_manifest('ar_loc_base')
import rospy
from geometry_msgs.msg import Twist
import numpy
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin

prefix=["FL","FR","CL","CR","RL","RR"]

class RoverMotors:
    def __init__(self):
        self.steering={}
        self.drive={}
        for k in prefix:
            self.steering[k]=0.0
            self.drive[k]=0.0
    def copy(self,value):
        for k in prefix:
            self.steering[k]=value.steering[k]
            self.drive[k]=value.drive[k]

class DriveConfiguration:
    def __init__(self,radius,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius


class RoverKinematics:
    def __init__(self):
        self.X = numpy.asmatrix(numpy.zeros((3,1)))
        self.motor_state = RoverMotors()
        self.first_run = True

    def twist_to_motors(self, twist, drive_cfg, skidsteer=False):
        motors = RoverMotors()
        # print "-"*32
        if skidsteer:
            for k in drive_cfg.keys():
                # First compute the speed from each wheel 
                #     V_wheel = V_body + Omega x R
                vw_x = twist.linear.x - 2*twist.angular.z*drive_cfg[k].y
                vw_y = 0
                motors.steering[k] = 0
                motors.drive[k] = vw_x / drive_cfg[k].radius
        else:
            for k in drive_cfg.keys():
                # First compute the speed from each wheel 
                #     V_wheel = V_body + Omega x R
                vw_x = twist.linear.x - twist.angular.z*drive_cfg[k].y
                vw_y = twist.linear.y + twist.angular.z*drive_cfg[k].x
                motors.steering[k] = atan2(vw_y,vw_x); 
                motors.drive[k] = hypot(vw_y,vw_x) / drive_cfg[k].radius
                if motors.steering[k] > pi/2:
                    motors.steering[k] -= pi
                    motors.drive[k] = -motors.drive[k]
                if motors.steering[k] <-pi/2:
                    motors.steering[k] += pi
                    motors.drive[k] = -motors.drive[k]
                # print "%s: T %.2f %.2f %.2f V %.2f %.2f S %.2f D %.2f" % \
                #     (k,drive_pose[k]["x"],drive_pose[k]["y"],drive_pose[k]["z"],\
                #     vw_x,vw_y,motors.steering[k]*180./pi,motors.drive[k])
        return motors

    def prepare_inversion_matrix(self,drive_cfg):
        W = numpy.asmatrix(numpy.zeros((len(prefix)*2,3)))
        for i in range(len(prefix)):
            k = prefix[i]
            # prepare the least-square matrices
            W[2*i+0,0] = 1; W[2*i+0,1] = 0; W[2*i+0,2] = -drive_cfg[k].y; 
            W[2*i+1,0] = 0; W[2*i+1,1] = 1; W[2*i+1,2] = +drive_cfg[k].x; 
        return pinv(W)

    def prepare_displacement_matrix(self, motor_state_t1, motor_state_t2, drive_cfg):
        # then compute odometry using least square
        S = numpy.asmatrix(numpy.zeros((len(prefix)*2,1)))
        for i in range(len(prefix)):
            k = prefix[i]
            # compute differentials
            beta = (motor_state_t1.steering[k]+motor_state_t2.steering[k])/2
            ds = (motor_state_t2.drive[k] - motor_state_t1.drive[k]) % (2*pi)
            if ds>pi:
                ds -= 2*pi
            ds *= drive_cfg[k].radius
            S[2*i+0,0] = ds*cos(beta)
            S[2*i+1,0] = ds*sin(beta)
        return S

    def compute_displacement(self, motor_state, drive_cfg):
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            return numpy.asmatrix(numpy.zeros((3,1)))
        # print "-"*32
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
        dX = iW * S 
        self.motor_state.copy(motor_state)
        return dX

    def integrate_odometry(self, motor_state, drive_cfg):
        dX = self.compute_displacement(motor_state,drive_cfg)
        self.X[0,0] += dX[0,0]*cos(self.X[2,0]) - dX[1,0]*sin(self.X[2,0])
        self.X[1,0] += dX[0,0]*sin(self.X[2,0]) + dX[1,0]*cos(self.X[2,0])
        self.X[2,0] += dX[2,0]
        return self.X



