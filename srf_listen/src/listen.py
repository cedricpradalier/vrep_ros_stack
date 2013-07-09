#!/usr/bin/python
PKG = 'srf_listen'
NAME = 'srf_listen'
import roslib; roslib.load_manifest(PKG)
import rospy

from std_msgs.msg import Float32
from srf_listen.msg import sonar_array

import serial;
rospy.init_node(NAME, anonymous=False)

ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=9600
)
ser.open();

data_pub = rospy.Publisher("~us",sonar_array)

def filter(x):
    if x == 0xFFFF:
        return 0
    if x == 0x8080:
        return 0
    return x

while True:
    line = ser.readline();
    if line.strip() == "Timeout":
        continue
    l = line.split(" ");
    num = int(l[0])
    S = sonar_array()
    S.header.stamp=rospy.Time.now()
    S.num = num
    S.data_us = [filter(int(s,base=16)) for s in l[1:5]]
    data_pub.publish(S)

ser.close()

