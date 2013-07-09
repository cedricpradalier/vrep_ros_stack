#ifndef SIM_TASKS_ENV_H
#define SIM_TASKS_ENV_H

#include <ros/ros.h>
#include "task_manager_lib/TaskDefinition.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace floor_nav {
    class SimTasksEnv: public task_manager_lib::TaskEnvironment
    {
        protected:
            ros::NodeHandle nh;
            bool paused;
            ros::Subscriber muxSub;
            ros::Subscriber pointCloudSub;
            ros::Publisher velPub;
            ros::ServiceClient muxClient;
            tf::TransformListener listener;

            void muxCallback(const std_msgs::String::ConstPtr& msg) ;

            void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr msg) ;

            bool manualControl;
            std::string joystick_topic;
            std::string auto_topic;
            std::string body_name;
            pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
            
            // Specific variable for lake circumnavigation
            geometry_msgs::Pose2D finishLine2D;

        public:
            SimTasksEnv(ros::NodeHandle & nh);
            ~SimTasksEnv() {};

            ros::NodeHandle & getNodeHandle() {return nh;}

            geometry_msgs::Pose2D getPose2D() const ; 

            geometry_msgs::Pose getPose() const ;

            geometry_msgs::PoseStamped getPoseStamped() const  ;

            pcl::PointCloud<pcl::PointXYZRGB> getPointCloud() const {return pointCloud;}

            void publishVelocity(double linear, double angular) ;

            void setManualControl();
            void setComputerControl();
            bool getManualControl() const {return manualControl;}

            // Specific variable for lake circumnavigation
            void setFinishLine2D(geometry_msgs::Pose2D pose) {finishLine2D = pose;}
            geometry_msgs::Pose2D getFinishLine2D() const {return finishLine2D;}
        public: // To make point cloud work on 32bit system
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

};

#endif // SIM_TASKS_ENV_H
