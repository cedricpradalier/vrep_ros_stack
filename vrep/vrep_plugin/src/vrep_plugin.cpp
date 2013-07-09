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

#include "v_repLib.h"
#include <boost/lexical_cast.hpp>
#include "vrep_plugin/vrep_plugin.h"
#include "vrep_plugin/ROS_server.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <iostream>
#include <opencv2/opencv.hpp>

#define PLUGIN_VERSION 2

#define LUA_ENABLE_PUBLISHER			"simExtROS_enablePublisher"
#define LUA_ENABLE_PUBLISHER_TIPS 	"string topicName=" LUA_ENABLE_PUBLISHER "(string topicName,number queueSize,number rosStreamCmd,number auxInt1,number auxInt2,string auxString)"

#define LUA_DISABLE_PUBLISHER			"simExtROS_disablePublisher"
#define LUA_DISABLE_PUBLISHER_TIPS 	"number referenceCounter=" LUA_DISABLE_PUBLISHER "(string topicName)"


#define LUA_ENABLE_SUBSCRIBER			"simExtROS_enableSubscriber"
#define LUA_ENABLE_SUBSCRIBER_TIPS 	"number subscriberID=" LUA_ENABLE_SUBSCRIBER "(string topicName,number queueSize,number rosStreamCmd,number auxInt1,number auxInt2,string auxString)"

#define LUA_DISABLE_SUBSCRIBER			"simExtROS_disableSubscriber"
#define LUA_DISABLE_SUBSCRIBER_TIPS 	"boolean result=" LUA_DISABLE_SUBSCRIBER "(number subscriberID)"

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

void LUA_ENABLE_PUBLISHER_CALLBACK(SLuaCallBack* p)
{ 
	simLockInterface(1);
	std::string effectiveTopicName;
	if (p->inputArgCount>=6)
	{ 
		// Ok, we have at least 5 input argument
		if (	(p->inputArgTypeAndSize[0*2+0]==sim_lua_arg_string) && 
				(p->inputArgTypeAndSize[1*2+0]==sim_lua_arg_int) &&
				(p->inputArgTypeAndSize[2*2+0]==sim_lua_arg_int) &&
				(p->inputArgTypeAndSize[3*2+0]==sim_lua_arg_int) &&
				(p->inputArgTypeAndSize[4*2+0]==sim_lua_arg_int) &&
				(p->inputArgTypeAndSize[5*2+0]==sim_lua_arg_string) )
		{ // the input arguments seem ok
			std::string topicName(p->inputChar);
			std::string auxString(p->inputChar+strlen(p->inputChar)+1);
			int queueSize=p->inputInt[0];
			int streamCmd=p->inputInt[1];
			int auxInt1=p->inputInt[2];
			int auxInt2=p->inputInt[3];

			int errorModeSaved;
			simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
			simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

			effectiveTopicName=ROS_server::addPublisher(topicName.c_str(),queueSize,streamCmd,auxInt1,auxInt2,auxString.c_str());

			simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings

			if (effectiveTopicName.length()==0)
				simSetLastError(LUA_ENABLE_PUBLISHER, "Topic could not be published."); // output an error
		}
		else
			simSetLastError(LUA_ENABLE_PUBLISHER, "Wrong argument type/size."); // output an error
	}
	else
		simSetLastError(LUA_ENABLE_PUBLISHER, "Wrong number of arguments."); // output an error

	// Now we prepare the return value:
	if (effectiveTopicName.length()==0)
		p->outputArgCount=0;
	else
	{
		p->outputArgCount=1;
		p->outputArgTypeAndSize=(int*)simCreateBuffer(p->outputArgCount*2*sizeof(int)); 
		p->outputArgTypeAndSize[2*0+0]=sim_lua_arg_string;	
		p->outputArgTypeAndSize[2*0+1]=1;
		p->outputChar=(simChar*)simCreateBuffer(effectiveTopicName.length()+1);
		for (unsigned int i=0;i<effectiveTopicName.length();i++)
			p->outputChar[i]=effectiveTopicName[i];
		p->outputChar[effectiveTopicName.length()]=0;
	}
	
	simLockInterface(0);
}

void LUA_DISABLE_PUBLISHER_CALLBACK(SLuaCallBack* p)
{ 
	simLockInterface(1);
	int result=-1;
	if (p->inputArgCount>=1)
	{ 
		// Ok, we have at least 1 input argument
		if (p->inputArgTypeAndSize[0*2+0]==sim_lua_arg_string)
		{ // the input argument seem ok
			std::string topicName(p->inputChar);

			int errorModeSaved;
			simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
			simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

			result=ROS_server::removePublisher(topicName.c_str(),false);

			simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings

			if (result==-1)
				simSetLastError(LUA_DISABLE_PUBLISHER, "Topic could not be unpublished."); // output an error
		}
		else
			simSetLastError(LUA_DISABLE_PUBLISHER, "Wrong argument type/size."); // output an error
	}
	else
		simSetLastError(LUA_DISABLE_PUBLISHER, "Wrong number of arguments."); // output an error

	// Now we prepare the return value:
	p->outputArgCount=1;
	p->outputArgTypeAndSize=(int*)simCreateBuffer(p->outputArgCount*2*sizeof(int)); 
	p->outputArgTypeAndSize[2*0+0]=sim_lua_arg_int;	
	p->outputArgTypeAndSize[2*0+1]=1;
	p->outputInt=(simInt*)simCreateBuffer(sizeof(simInt));
	p->outputInt[0]=result;
	
	simLockInterface(0);
}


void LUA_ENABLE_SUBSCRIBER_CALLBACK(SLuaCallBack* p)
{ 
	simLockInterface(1);
	int result=-1;
	if (p->inputArgCount>=6)
	{ 
		// Ok, we have at least 5 input argument
		if (	(p->inputArgTypeAndSize[0*2+0]==sim_lua_arg_string) && 
				(p->inputArgTypeAndSize[1*2+0]==sim_lua_arg_int) &&
				(p->inputArgTypeAndSize[2*2+0]==sim_lua_arg_int) &&
				(p->inputArgTypeAndSize[3*2+0]==sim_lua_arg_int) &&
				(p->inputArgTypeAndSize[4*2+0]==sim_lua_arg_int) &&
				(p->inputArgTypeAndSize[5*2+0]==sim_lua_arg_string) )
		{ // the input arguments seem ok
			std::string topicName(p->inputChar);
			std::string auxString(p->inputChar+strlen(p->inputChar)+1);
			int queueSize=p->inputInt[0];
			int streamCmd=p->inputInt[1];
			int auxInt1=p->inputInt[2];
			int auxInt2=p->inputInt[3];

			int errorModeSaved;
			simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
			simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

			result=ROS_server::addSubscriber(topicName.c_str(),queueSize,streamCmd,auxInt1,auxInt2,auxString.c_str());

			simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings

			if (result==-1)
				simSetLastError(LUA_ENABLE_SUBSCRIBER, "Topic could not be subscribed."); // output an error
		}
		else
			simSetLastError(LUA_ENABLE_SUBSCRIBER, "Wrong argument type/size."); // output an error
	}
	else
		simSetLastError(LUA_ENABLE_SUBSCRIBER, "Wrong number of arguments."); // output an error

	// Now we prepare the return value:
	p->outputArgCount=1;
	p->outputArgTypeAndSize=(int*)simCreateBuffer(p->outputArgCount*2*sizeof(int)); 
	p->outputArgTypeAndSize[2*0+0]=sim_lua_arg_int;	
	p->outputArgTypeAndSize[2*0+1]=1;
	p->outputInt=(simInt*)simCreateBuffer(sizeof(simInt));
	p->outputInt[0]=result;
	
	simLockInterface(0);
}

void LUA_DISABLE_SUBSCRIBER_CALLBACK(SLuaCallBack* p)
{ 
	simLockInterface(1);
	bool result=false;
	if (p->inputArgCount>=1)
	{ 
		// Ok, we have at least 1 input argument
		if (p->inputArgTypeAndSize[0*2+0]==sim_lua_arg_int)
		{ // the input argument seem ok

			int errorModeSaved;
			simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
			simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

			result=ROS_server::removeSubscriber(p->inputInt[0]);

			simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings

			if (!result)
				simSetLastError(LUA_DISABLE_SUBSCRIBER, "Topic could not be unsubscribed."); // output an error
		}
		else
			simSetLastError(LUA_DISABLE_SUBSCRIBER, "Wrong argument type/size."); // output an error
	}
	else
		simSetLastError(LUA_DISABLE_SUBSCRIBER, "Wrong number of arguments."); // output an error

	// Now we prepare the return value:
	p->outputArgCount=1;
	p->outputArgTypeAndSize=(int*)simCreateBuffer(p->outputArgCount*2*sizeof(int)); 
	p->outputArgTypeAndSize[2*0+0]=sim_lua_arg_bool;	
	p->outputArgTypeAndSize[2*0+1]=1;
	p->outputBool=(simBool*)simCreateBuffer(sizeof(simBool));
	p->outputBool[0]=result;
	
	simLockInterface(0);
}


// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
	// Dynamically load and bind V-REP functions:
	// ******************************************
	// 1. Figure out this plugin's directory:
	char curDirAndFile[1024];
	getcwd(curDirAndFile, sizeof(curDirAndFile));

	std::string currentDirAndPath(curDirAndFile);
	// 2. Append the V-REP library's name:
	std::string temp(currentDirAndPath);
	temp+="/libv_rep.so";

	// 3. Load the V-REP library:
	vrepLib=loadVrepLibrary(temp.c_str());
	if (vrepLib==NULL)
	{
		std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'ROS' plugin.\n";
		return(0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
		std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'ROS' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************

	// Check the version of V-REP:
	// ******************************************
	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
	if (vrepVer<20605) // if V-REP version is smaller than 2.06.04
	{
		std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'ROS' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************
	

	
	// Initialize the ROS part:
	if(!ROS_server::initialize()) 
	{
		std::cout << "ROS master is not running. Cannot start 'ROS' plugin.\n";
		return (0); //If the master is not running then the plugin is not loaded.
	}
	
	simLockInterface(1);

	int enablePublisherArgs[]={6,sim_lua_arg_string,sim_lua_arg_int,sim_lua_arg_int,sim_lua_arg_int,sim_lua_arg_int,sim_lua_arg_string};
	simRegisterCustomLuaFunction(LUA_ENABLE_PUBLISHER,LUA_ENABLE_PUBLISHER_TIPS,enablePublisherArgs,LUA_ENABLE_PUBLISHER_CALLBACK);

	int disablePublisherArgs[]={1,sim_lua_arg_string};
	simRegisterCustomLuaFunction(LUA_DISABLE_PUBLISHER,LUA_DISABLE_PUBLISHER_TIPS,disablePublisherArgs,LUA_DISABLE_PUBLISHER_CALLBACK);


	int enableSubscriberArgs[]={6,sim_lua_arg_string,sim_lua_arg_int,sim_lua_arg_int,sim_lua_arg_int,sim_lua_arg_int,sim_lua_arg_string};
	simRegisterCustomLuaFunction(LUA_ENABLE_SUBSCRIBER,LUA_ENABLE_SUBSCRIBER_TIPS,enableSubscriberArgs,LUA_ENABLE_SUBSCRIBER_CALLBACK);

	int disableSubscriberArgs[]={1,sim_lua_arg_int};
	simRegisterCustomLuaFunction(LUA_DISABLE_SUBSCRIBER,LUA_DISABLE_SUBSCRIBER_TIPS,disableSubscriberArgs,LUA_DISABLE_SUBSCRIBER_CALLBACK);


	// Publisher constants:
	simRegisterCustomLuaVariable("simros_strmcmd_get_object_selection",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_object_selection))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_array_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_array_parameter))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_boolean_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_boolean_parameter))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_dialog_result",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_dialog_result))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_floating_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_floating_parameter))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_integer_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_integer_parameter))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_joint_state",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_joint_state))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_object_pose",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_object_pose))).c_str());
//	simRegisterCustomLuaVariable("simros_strmcmd_get_object_quaternion",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_object_quaternion))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_object_parent",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_object_parent))).c_str());
//	simRegisterCustomLuaVariable("simros_strmcmd_get_object_position",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_object_position))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_objects",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_objects))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_string_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_string_parameter))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_ui_event_button",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_ui_event_button))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_vision_sensor_depth_buffer",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_vision_sensor_depth_buffer))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_vision_sensor_image",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_vision_sensor_image))).c_str());
//	simRegisterCustomLuaVariable("simros_strmcmd_get_joint_force",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_joint_force))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_read_collision",(boost::lexical_cast<std::string>(int(simros_strmcmd_read_collision))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_read_distance",(boost::lexical_cast<std::string>(int(simros_strmcmd_read_distance))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_read_force_sensor",(boost::lexical_cast<std::string>(int(simros_strmcmd_read_force_sensor))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_read_proximity_sensor",(boost::lexical_cast<std::string>(int(simros_strmcmd_read_proximity_sensor))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_read_vision_sensor",(boost::lexical_cast<std::string>(int(simros_strmcmd_read_vision_sensor))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_object_float_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_object_float_parameter))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_object_int_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_object_int_parameter))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_ui_button_property",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_ui_button_property))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_ui_slider",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_ui_slider))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_float_signal",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_float_signal))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_integer_signal",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_integer_signal))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_string_signal",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_string_signal))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_and_clear_string_signal",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_and_clear_string_signal))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_transform",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_transform))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_range_finder_data",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_range_finder_data))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_depth_sensor_data",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_depth_sensor_data))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_vision_sensor_info",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_vision_sensor_info))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_twist_status",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_twist_status))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_get_object_group_data",(boost::lexical_cast<std::string>(int(simros_strmcmd_get_object_group_data))).c_str());

	// Subscriber constants:
	simRegisterCustomLuaVariable("simros_strmcmd_add_status_bar_message",(boost::lexical_cast<std::string>(int(simros_strmcmd_add_status_bar_message))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_object_selection",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_object_selection))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_auxiliary_console_print",(boost::lexical_cast<std::string>(int(simros_strmcmd_auxiliary_console_print))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_array_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_array_parameter))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_boolean_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_boolean_parameter))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_floating_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_floating_parameter))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_integer_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_integer_parameter))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_joint_force",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_joint_force))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_joint_position",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_joint_position))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_joint_target_position",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_joint_target_position))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_joint_target_velocity",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_joint_target_velocity))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_vision_sensor_image",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_vision_sensor_image))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_object_float_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_object_float_parameter))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_object_int_parameter",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_object_int_parameter))).c_str());
//	simRegisterCustomLuaVariable("simros_strmcmd_set_object_orientation",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_object_orientation))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_object_position",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_object_position))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_object_pose",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_object_pose))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_joint_state",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_joint_state))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_object_quaternion",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_object_quaternion))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_ui_button_label",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_ui_button_label))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_ui_button_property",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_ui_button_property))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_ui_slider",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_ui_slider))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_clear_float_signal",(boost::lexical_cast<std::string>(int(simros_strmcmd_clear_float_signal))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_clear_integer_signal",(boost::lexical_cast<std::string>(int(simros_strmcmd_clear_integer_signal))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_clear_string_signal",(boost::lexical_cast<std::string>(int(simros_strmcmd_clear_string_signal))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_float_signal",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_float_signal))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_integer_signal",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_integer_signal))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_string_signal",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_string_signal))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_append_string_signal",(boost::lexical_cast<std::string>(int(simros_strmcmd_append_string_signal))).c_str());
	simRegisterCustomLuaVariable("simros_strmcmd_set_twist_command",(boost::lexical_cast<std::string>(int(simros_strmcmd_set_twist_command))).c_str());

	simLockInterface(0);

	return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
	ROS_server::shutDown();	// shutdown the ROS_server
	unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ 
	// This is called quite often. Just watch out for messages/events you want to handle
	// Keep following 5 lines at the beginning and unchanged:
	simLockInterface(1);
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
	void* retVal=NULL;

	// Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here:

	if (message==sim_message_eventcallback_instancepass)
	{ 
		// This message is sent each time the scene was rendered (well, shortly after) (very often)
		// When a simulation is not running, but you still need to execute some commands, then put some code here
		ROS_server::instancePass();
	}

	if (message==sim_message_eventcallback_mainscriptabouttobecalled)
	{ 
		// Main script is about to be run (only called while a simulation is running (and not paused!))
		//
		// This is a good location to execute simulation commands
		
		if (ROS_server::mainScriptAboutToBeCalled())
			replyData[0]=0; // this tells V-REP that we don't wanna execute the main script
	}

	if (message==sim_message_eventcallback_simulationabouttostart)
	{ 
	    // Simulation is about to start
		
		ROS_server::simulationAboutToStart();
	}

	if (message==sim_message_eventcallback_simulationended)
	{ 
		// Simulation just ended
		
		ROS_server::simulationEnded();
	}

	// Keep following unchanged:
	simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
	simLockInterface(0);
	return(retVal);
}

