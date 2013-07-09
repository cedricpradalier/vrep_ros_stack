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

#include "vrep_plugin/vrepSubscriber.h"
#include "v_repLib.h"

CSubscriberData::CSubscriberData(ros::NodeHandle* node,const char* _topicName,int queueSize,int _streamCmd,int _auxInt1,int _auxInt2,const char* _auxString,image_transport::ImageTransport* images_streamer[1],int& imgStreamerCnt)
{
	cmdID=_streamCmd;
	auxInt1=_auxInt1;
	auxInt2=_auxInt2;
	auxStr=_auxString;
	topicName=_topicName;
	isValid=false;

	if (cmdID==simros_strmcmd_add_status_bar_message)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::addStatusbarMessageCallback,this);
		isValid=true;
	}

	if (cmdID==simros_strmcmd_auxiliary_console_print)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::auxiliaryConsolePrintCallback,this);
		isValid=true;
	}

	if (cmdID==simros_strmcmd_clear_float_signal)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::clearFloatSignalCallback,this);
		isValid=true;
	}

	if (cmdID==simros_strmcmd_clear_integer_signal)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::clearIntegerSignalCallback,this);
		isValid=true;
	}

	if (cmdID==simros_strmcmd_clear_string_signal)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::clearStringSignalCallback,this);
		isValid=true;
	}

	if (cmdID==simros_strmcmd_set_array_parameter)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setArrayParameterCallback,this);
		isValid=true;
	}

	if (cmdID==simros_strmcmd_set_boolean_parameter)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setBooleanParameterCallback,this);
		isValid=true;
	}

	if (cmdID==simros_strmcmd_set_floating_parameter)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setFloatingParameterCallback,this);
		isValid=true;
	}

	if (cmdID==simros_strmcmd_set_integer_parameter)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setIntegerParameterCallback,this);
		isValid=true;
	}

	if (cmdID==simros_strmcmd_set_float_signal)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setFloatSignalCallback,this);
		isValid=true;
	}
	if (cmdID==simros_strmcmd_set_integer_signal)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setIntegerSignalCallback,this);
		isValid=true;
	}
	if (cmdID==simros_strmcmd_set_string_signal)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setStringSignalCallback,this);
		isValid=true;
	}
	if (cmdID==simros_strmcmd_append_string_signal)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::appendStringSignalCallback,this);
		isValid=true;
	}

	if (cmdID==simros_strmcmd_set_joint_force)
	{
		if (simGetObjectType(auxInt1)==sim_object_joint_type)
		{
			generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setJointForceCallback,this);
			isValid=true;
		}
	}

	if (cmdID==simros_strmcmd_set_joint_position)
	{
		if (simGetObjectType(auxInt1)==sim_object_joint_type)
		{
			generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setJointPositionCallback,this);
			isValid=true;
		}
	}

	if (cmdID==simros_strmcmd_set_joint_target_position)
	{
		if (simGetObjectType(auxInt1)==sim_object_joint_type)
		{
			generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setJointTargetPositionCallback,this);
			isValid=true;
		}
	}

	if (cmdID==simros_strmcmd_set_joint_target_velocity)
	{
		if (simGetObjectType(auxInt1)==sim_object_joint_type)
		{
			generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setJointTargetVelocityCallback,this);
			isValid=true;
		}
	}
	if (cmdID==simros_strmcmd_set_twist_command)
	{
        generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setTwistCommandCallback,this);
        isValid=true;
	}
	if (cmdID==simros_strmcmd_set_object_float_parameter)
	{
		if (simGetObjectType(auxInt1)!=-1)
		{
			generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setObjectFloatParameterCallback,this);
			isValid=true;
		}
	}

	if (cmdID==simros_strmcmd_set_object_int_parameter)
	{
		if (simGetObjectType(auxInt1)!=-1)
		{
			generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setObjectIntParameterCallback,this);
			isValid=true;
		}
	}

	if (cmdID==simros_strmcmd_set_object_pose)
	{
		if ( (simGetObjectType(auxInt1)!=-1)&&( (simGetObjectType(auxInt2)!=-1)||(auxInt2==sim_handle_parent)||(auxInt2==-1) ) )
		{
			generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setObjectPoseCallback,this);
			isValid=true;
		}
	}

	if (cmdID==simros_strmcmd_set_object_position)
	{
		if ( (simGetObjectType(auxInt1)!=-1)&&( (simGetObjectType(auxInt2)!=-1)||(auxInt2==sim_handle_parent)||(auxInt2==-1) ) )
		{
			generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setObjectPositionCallback,this);
			isValid=true;
		}
	}

	if (cmdID==simros_strmcmd_set_object_quaternion)
	{
		if ( (simGetObjectType(auxInt1)!=-1)&&( (simGetObjectType(auxInt2)!=-1)||(auxInt2==sim_handle_parent)||(auxInt2==-1) ) )
		{
			generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setObjectQuaternionCallback,this);
			isValid=true;
		}
	}

	if (cmdID==simros_strmcmd_set_object_selection)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setObjectSelectionCallback,this);
		isValid=true;
	}

	if (cmdID==simros_strmcmd_set_ui_button_label)
	{
		if (simGetUIButtonProperty(auxInt1,auxInt2)!=-1)
		{
			generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setUIButtonLabelCallback,this);
			isValid=true;
		}
	}

	if (cmdID==simros_strmcmd_set_ui_button_property)
	{
		if (simGetUIButtonProperty(auxInt1,auxInt2)!=-1)
		{
			generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setUIButtonPropertyCallback,this);
			isValid=true;
		}
	}

	if (cmdID==simros_strmcmd_set_ui_slider)
	{
		if (simGetUIButtonProperty(auxInt1,auxInt2)!=-1)
		{
			generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setUISlider,this);
			isValid=true;
		}
	}

	if (cmdID==simros_strmcmd_set_vision_sensor_image)
	{
		if (simGetObjectType(auxInt1)==sim_object_visionsensor_type)
		{
			if (imgStreamerCnt==0)
				images_streamer[0]= new image_transport::ImageTransport(*node);
			imageSubscriber=images_streamer[0]->subscribe(topicName,queueSize,&CSubscriberData::setVisionSensorImageCallback,this);
			imgStreamerCnt++;
			isValid=true;
		}
	}

	if (cmdID==simros_strmcmd_set_joint_state)
	{
		generalSubscriber=node->subscribe(topicName,queueSize,&CSubscriberData::setJointStateCallback,this);
		isValid=true;
	}

}

CSubscriberData::~CSubscriberData()
{
}

bool CSubscriberData::getIsValid()
{
	return(isValid);
}

void CSubscriberData::setSubscriberID(int id)
{
	subscriberID=id;
}

int CSubscriberData::getSubscriberID()
{
	return(subscriberID);
}

void CSubscriberData::shutDownSubscriber()
{
	if (isValid)
	{
		if (cmdID==simros_strmcmd_set_vision_sensor_image)
			shutDownImageSubscriber();
		else
			shutDownGeneralSubscriber();
	}
}

void CSubscriberData::shutDownGeneralSubscriber()
{
	generalSubscriber.shutdown();
	isValid=false;
}

void CSubscriberData::shutDownImageSubscriber()
{
	imageSubscriber.shutdown();
	isValid=false;
}

void CSubscriberData::addStatusbarMessageCallback(const std_msgs::String::ConstPtr& msg)
{
	if (simAddStatusbarMessage(msg->data.c_str())==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::auxiliaryConsolePrintCallback(const std_msgs::String::ConstPtr& txt)
{
	if (txt->data.length()==0)
	{
		if (simAuxiliaryConsolePrint(auxInt1,NULL)<=0)
			shutDownGeneralSubscriber();
	}
	else
	{
		if (simAuxiliaryConsolePrint(auxInt1,txt->data.c_str())<=0)
			shutDownGeneralSubscriber();
	}
}

void CSubscriberData::clearFloatSignalCallback(const std_msgs::UInt8::ConstPtr& options)
{
	if (options->data==0)
		simClearFloatSignal(auxStr.c_str());
	else
		simClearFloatSignal(NULL);
}

void CSubscriberData::clearIntegerSignalCallback(const std_msgs::UInt8::ConstPtr& options)
{
	if (options->data==0)
		simClearIntegerSignal(auxStr.c_str());
	else
		simClearIntegerSignal(NULL);
}

void CSubscriberData::clearStringSignalCallback(const std_msgs::UInt8::ConstPtr& options)
{
	if (options->data==0)
		simClearStringSignal(auxStr.c_str());
	else
		simClearStringSignal(NULL);
}

void CSubscriberData::setArrayParameterCallback(const geometry_msgs::Point32::ConstPtr& param)
{
	float v[3]={param->x,param->y,param->z};
	if (simSetArrayParameter(auxInt1,(void*)v)==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setBooleanParameterCallback(const std_msgs::UInt8::ConstPtr& param)
{
	if (simSetBooleanParameter(auxInt1,param->data)==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setFloatingParameterCallback(const std_msgs::Float32::ConstPtr& param)
{
	if (simSetFloatingParameter(auxInt1,param->data)==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setIntegerParameterCallback(const std_msgs::Int32::ConstPtr& param)
{
	if (simSetIntegerParameter(auxInt1,param->data)==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setFloatSignalCallback(const std_msgs::Float32::ConstPtr& sig)
{
	simSetFloatSignal(auxStr.c_str(),sig->data);
}

void CSubscriberData::setIntegerSignalCallback(const std_msgs::Int32::ConstPtr& sig)
{
	simSetIntegerSignal(auxStr.c_str(),sig->data);
}

void CSubscriberData::setJointForceCallback(const std_msgs::Float64::ConstPtr& force)
{
	if (simSetJointForce(auxInt1,(float)force->data)==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setJointPositionCallback(const std_msgs::Float64::ConstPtr& pos)
{
	if (simSetJointPosition(auxInt1,(float)pos->data)==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setJointTargetPositionCallback(const std_msgs::Float64::ConstPtr& pos)
{
	if (simSetJointTargetPosition(auxInt1,(float)pos->data)==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setJointTargetVelocityCallback(const std_msgs::Float64::ConstPtr& vel)
{
	if (simSetJointTargetVelocity(auxInt1,(float)vel->data)==-1)
		shutDownGeneralSubscriber();
}
void CSubscriberData::setTwistCommandCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
    simFloat command[6] = {vel->linear.x,vel->linear.y,vel->linear.z,vel->angular.x,vel->angular.y,vel->angular.z};
	if (simSetStringSignal(auxStr.c_str(),(simChar*)command,6*sizeof(simFloat))==-1)
		shutDownGeneralSubscriber();
}
void CSubscriberData::setObjectFloatParameterCallback(const std_msgs::Float32::ConstPtr& param)
{
	if (simSetObjectFloatParameter(auxInt1,auxInt2,param->data)<=0)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setObjectIntParameterCallback(const std_msgs::Int32::ConstPtr& param)
{
	if (simSetObjectIntParameter(auxInt1,auxInt2,param->data)<=0)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setObjectPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	float p[3]={(float)pose->pose.position.x,(float)pose->pose.position.y,(float)pose->pose.position.z};
	float q[4]={(float)pose->pose.orientation.x,(float)pose->pose.orientation.y,(float)pose->pose.orientation.z,(float)pose->pose.orientation.w};
	if (simSetObjectPosition(auxInt1,auxInt2,p)==-1)
		shutDownGeneralSubscriber();
	else
		simSetObjectQuaternion(auxInt1,auxInt2,q);
}

void CSubscriberData::setObjectPositionCallback(const geometry_msgs::Point::ConstPtr& pos)
{
	float v[3]={(float)pos->x,(float)pos->y,(float)pos->z};
	if (simSetObjectPosition(auxInt1,auxInt2,v)==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setObjectQuaternionCallback(const geometry_msgs::Quaternion::ConstPtr& quaternion)
{
	float v[4]={(float)quaternion->x,(float)quaternion->y,(float)quaternion->z,(float)quaternion->w};
	if (simSetObjectQuaternion(auxInt1,auxInt2,v)==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setObjectSelectionCallback(const std_msgs::Int32MultiArray::ConstPtr& objHandles)
{
	simRemoveObjectFromSelection(sim_handle_all,0);
	int cnt=objHandles->data.size();
	for (int i=0;i<cnt;i++)
		simAddObjectToSelection(sim_handle_single,objHandles->data[i]);
}

void CSubscriberData::setStringSignalCallback(const std_msgs::String::ConstPtr& sig)
{
	simSetStringSignal(auxStr.c_str(),&sig->data[0],sig->data.length());
}

void CSubscriberData::appendStringSignalCallback(const std_msgs::String::ConstPtr& sig)
{
	std::string theNewString;
	int stringLength;
	char* stringSignal=simGetStringSignal(auxStr.c_str(),&stringLength);
	if (stringSignal!=NULL)
	{
		theNewString=std::string(stringSignal,stringLength);
		simReleaseBuffer(stringSignal);
	}
	theNewString+=std::string(&sig->data[0],sig->data.length());
	simSetStringSignal(auxStr.c_str(),theNewString.c_str(),theNewString.length());
}

void CSubscriberData::setUIButtonLabelCallback(const std_msgs::String::ConstPtr& label)
{
	if (simSetUIButtonLabel(auxInt1,auxInt2,label->data.c_str(),NULL)==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setUIButtonPropertyCallback(const std_msgs::Int32::ConstPtr& prop)
{
	if (simSetUIButtonProperty(auxInt1,auxInt2,prop->data)==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setUISlider(const std_msgs::Int32::ConstPtr& pos)
{
	if (simSetUISlider(auxInt1,auxInt2,pos->data)==-1)
		shutDownGeneralSubscriber();
}

void CSubscriberData::setVisionSensorImageCallback(const sensor_msgs::Image::ConstPtr& image)
{
	int resol[2];
	int result=simGetVisionSensorResolution(auxInt1,resol);
	if (result!=-1)
	{
		if ((image->encoding==sensor_msgs::image_encodings::RGB8)&&(image->step==image->width*3)&&(int(image->height)==resol[1])&&(int(image->width)==resol[0]))
		{
			float* image_buff=new float[3*resol[0]*resol[1]];
			for(unsigned int i=0;i<image->height;i++)
			{
				int msg_idx=(image->height-i-1)*image->step;
				int buf_idx=i*image->step;
				for(unsigned int j=0;j<image->step;j++)
					image_buff[buf_idx+j]=float(image->data[msg_idx+j])/255.0f;
			}
			simSetVisionSensorImage(auxInt1,image_buff);
			delete[] image_buff;
		}
	}
	else
		shutDownImageSubscriber();
}

void CSubscriberData::setJointStateCallback(const vrep_common::JointSetStateData::ConstPtr& data)
{
	if ( (data->handles.data.size()>0)&&(data->handles.data.size()==data->setModes.data.size())&&(data->handles.data.size()==data->values.data.size()) )
	{
		for (unsigned int i=0;i<data->handles.data.size();i++)
		{
			int handle=data->handles.data[i];
			unsigned char setMode=data->setModes.data[i];
			float val=data->values.data[i];
			if (setMode==0)
				simSetJointPosition(handle,val);
			if (setMode==1)
				simSetJointTargetPosition(handle,val);
			if (setMode==2)
				simSetJointTargetVelocity(handle,val);
			if (setMode==3)
				simSetJointForce(handle,val);
		}
	}
}
