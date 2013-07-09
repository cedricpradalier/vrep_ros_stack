// This file is part of the ROS PLUGIN for V-REP
// 
// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// A big thanks to Svetlin Penkov for his precious help!
// 
// The ROS PLUGIN is licensed under the terms of GNU GPL:
// 
// -------------------------------------------------------------------
// The ROS PLUGIN is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// The ROS PLUGIN is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with the ROS PLUGIN.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.4 on July 8th 2013

#ifndef VREP_SUBSCRIBER_H
#define VREP_SUBSCRIBER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include "vrep_common/VisionSensorDepthBuff.h"
#include "vrep_common/ForceSensorData.h"
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VisionSensorData.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"


class CSubscriberData
{
public:
	CSubscriberData(ros::NodeHandle* node,const char* _topicName,int queueSize,int _streamCmd,int _auxInt1,int _auxInt2,const char* _auxString,image_transport::ImageTransport* images_streamer[1],int& imgStreamerCnt);
	virtual ~CSubscriberData();

	bool getIsValid();
	void setSubscriberID(int id);
	int getSubscriberID();
	void shutDownSubscriber();

protected:
	void shutDownGeneralSubscriber();
	void shutDownImageSubscriber();

	bool isValid;
	int cmdID;
	int auxInt1;
	int auxInt2;
	int subscriberID;
	std::string auxStr;
	std::string topicName;
	ros::Subscriber generalSubscriber;
	image_transport::Subscriber imageSubscriber;

public:
	void addStatusbarMessageCallback(const std_msgs::String::ConstPtr& msg);
	void auxiliaryConsolePrintCallback(const std_msgs::String::ConstPtr& txt);
	void clearFloatSignalCallback(const std_msgs::UInt8::ConstPtr& options);
	void clearIntegerSignalCallback(const std_msgs::UInt8::ConstPtr& options);
	void clearStringSignalCallback(const std_msgs::UInt8::ConstPtr& options);
	void setArrayParameterCallback(const geometry_msgs::Point32::ConstPtr& param);
	void setBooleanParameterCallback(const std_msgs::UInt8::ConstPtr& param);
	void setFloatingParameterCallback(const std_msgs::Float32::ConstPtr& param);
	void setIntegerParameterCallback(const std_msgs::Int32::ConstPtr& param);
	void setFloatSignalCallback(const std_msgs::Float32::ConstPtr& sig);
	void setIntegerSignalCallback(const std_msgs::Int32::ConstPtr& sig);
	void setJointForceCallback(const std_msgs::Float64::ConstPtr& force);
	void setJointPositionCallback(const std_msgs::Float64::ConstPtr& pos);
	void setJointTargetPositionCallback(const std_msgs::Float64::ConstPtr& pos);
	void setJointTargetVelocityCallback(const std_msgs::Float64::ConstPtr& vel);
	void setTwistCommandCallback(const geometry_msgs::Twist::ConstPtr& vel);
	void setObjectFloatParameterCallback(const std_msgs::Float32::ConstPtr& param);
	void setObjectIntParameterCallback(const std_msgs::Int32::ConstPtr& param);
	void setObjectPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
	void setObjectPositionCallback(const geometry_msgs::Point::ConstPtr& pos);
	void setObjectQuaternionCallback(const geometry_msgs::Quaternion::ConstPtr& quaternion);
	void setObjectSelectionCallback(const std_msgs::Int32MultiArray::ConstPtr& objHandles);
	void setStringSignalCallback(const std_msgs::String::ConstPtr& sig);
	void appendStringSignalCallback(const std_msgs::String::ConstPtr& sig);
	void setUIButtonLabelCallback(const std_msgs::String::ConstPtr& label);
	void setUIButtonPropertyCallback(const std_msgs::Int32::ConstPtr& prop);
	void setUISlider(const std_msgs::Int32::ConstPtr& pos);
	void setVisionSensorImageCallback(const sensor_msgs::Image::ConstPtr& image);
	void setJointStateCallback(const vrep_common::JointSetStateData::ConstPtr& data);
};

#endif
