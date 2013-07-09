#!/usr/bin/env python

"""
#include <ros/ros.h>
#include <turtlesim/Velocity.h>
#include <sensor_msgs/Joy.h>
"""

import roslib; roslib.load_manifest('test_joy')
import rospy
from turtlesim.msg import Velocity
from sensor_msgs.msg import Joy
from std_msgs.msg import String

"""
ros::NodeHandle nh_;

int linear_, angular_;
double l_scale_, a_scale_;
ros::Publisher vel_pub_;
ros::Subscriber joy_sub_;
"""


class Controler:
	def __init__(self):
		rospy.init_node('joy_test', anonymous=True)
		self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback)
		self.vel_pub = rospy.Publisher('turtle1/command_velocity', Velocity)
		self.vel_val = Velocity()
		self.linear = rospy.get_param('axis_linear', 1)
		self.angular = rospy.get_param('axis_angular', 3)
		self.l_scale = rospy.get_param('scale_linear', 1)
		self.a_scale = rospy.get_param('scale_angular', 1)

	def start(self):
		while not rospy.is_shutdown():
			rospy.loginfo(rospy.get_name() + " : " + str(self.vel_val.linear) + " ; " + str(self.vel_val.angular))
			self.vel_pub.publish(self.vel_val)
			rospy.sleep(0.1)

	def joyCallback(self,joy):
		self.vel_val.linear = self.l_scale*joy.axes[self.linear]
		self.vel_val.angular = self.a_scale*joy.axes[self.angular]
		

if __name__ == '__main__':
	c = Controler()
	c.start()

"""
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  turtlesim::Velocity vel;
  vel.angular = a_scale_*joy->axes[angular_];
  vel.linear = l_scale_*joy->axes[linear_];
  vel_pub_.publish(vel);
}
"""


"""
TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

}

	

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        str = "hello world %s" % rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
"""





