
#include "floor_nav/SimTasksEnv.h"
#include <topic_tools/MuxSelect.h>
#include <boost/algorithm/string.hpp>

using namespace floor_nav;

SimTasksEnv::SimTasksEnv(ros::NodeHandle & n) :
    nh(n), paused(false), manualControl(true), joystick_topic("/teleop/twistCommand"), auto_topic("/mux/autoCommand"), body_name("/bubbleRob")
{
    nh.getParam("joystick_topic",joystick_topic);
    nh.getParam("auto_topic",auto_topic);
    nh.getParam("body_name",body_name);

    muxClient = nh.serviceClient<topic_tools::MuxSelect>("/mux/select");

    muxSub = nh.subscribe("/mux/selected",1,&SimTasksEnv::muxCallback,this);
    pointCloudSub = nh.subscribe("/vrep/depthSensor",1,&SimTasksEnv::pointCloudCallback,this);
    velPub = nh.advertise<geometry_msgs::Twist>(auto_topic,1);
}

void SimTasksEnv::setManualControl()
{
    topic_tools::MuxSelect select;
    select.request.topic = joystick_topic;
    if (!muxClient.call(select)) {
        ROS_ERROR("setManualControl: Failed to call service /mux/select");
    }
}

void SimTasksEnv::setComputerControl()
{
    topic_tools::MuxSelect select;
    select.request.topic = auto_topic;
    if (!muxClient.call(select)) {
        ROS_ERROR("setComputerControl: Failed to call service /mux/select");
    }
}


geometry_msgs::Pose2D SimTasksEnv::getPose2D() const {
    geometry_msgs::Pose2D pose;
    tf::StampedTransform transform;
    try{
        listener.lookupTransform("/world",body_name, 
                ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    pose.theta = tf::getYaw(transform.getRotation());
    pose.x = transform.getOrigin().x();
    pose.y = transform.getOrigin().y();
    return pose;
}

geometry_msgs::Pose SimTasksEnv::getPose() const {
    geometry_msgs::Pose pose;
    tf::StampedTransform transform;
    try{
        listener.lookupTransform("/world",body_name, 
                ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    tf::quaternionTFToMsg(transform.getRotation(),pose.orientation);
    tf::pointTFToMsg(transform.getOrigin(),pose.position);
    return pose;
}

geometry_msgs::PoseStamped SimTasksEnv::getPoseStamped() const {
    geometry_msgs::PoseStamped pose;
    tf::StampedTransform transform;
    try{
        listener.lookupTransform("/world",body_name, 
                ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    tf::quaternionTFToMsg(transform.getRotation(),pose.pose.orientation);
    tf::pointTFToMsg(transform.getOrigin(),pose.pose.position);
    pose.header.stamp = transform.stamp_;
    return pose;
}

void SimTasksEnv::publishVelocity(double linear, double angular) {
    geometry_msgs::Twist cmd;
    if (paused) {
        cmd.linear.x = 0.;
        cmd.angular.z = 0.;
    } else {
        cmd.linear.x = linear;
        cmd.angular.z = angular;
    }
    velPub.publish(cmd);
}

void SimTasksEnv::muxCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == joystick_topic) {
        manualControl = true;
    } else if (msg->data == auto_topic) {
        manualControl = false;
    } else {
        ROS_ERROR("Received unknown mux selection: '%s'",msg->data.c_str());
    }
}

void SimTasksEnv::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr msg) {
    pcl::fromROSMsg(*msg, pointCloud);
}

