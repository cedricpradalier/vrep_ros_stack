
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


class BehaviorBase {
    protected:
        ros::Subscriber scanSub;
        ros::Publisher velPub;

        ros::NodeHandle nh;

        ros::Time last_sensor_time;
        std::map<int,double> sensor_map;
        geometry_msgs::Twist desired_speed;

        double angular_range;
        double safety_range;
        double dont_care_range;
        double max_linear_speed;
        double max_angular_speed;


        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            std::map<int,double>::iterator sensor_it;
            pcl::PointCloud<pcl::PointXYZ> lastpc;
            pcl::fromROSMsg(*msg, lastpc);
            last_sensor_time = ros::Time::now();
            // Convert the pointcloud into a discrete set of range measurement
            // separated by ANGULAR_RANGE
            sensor_map.clear();
            unsigned int n = lastpc.size();
            for (unsigned int i=0;i<n;i++) {
                double r = hypot(lastpc[i].y,lastpc[i].x);
                if (r < 1e-2) {
                    // bogus point, the laser did not return
                    continue;
                }
                double alpha = atan2(lastpc[i].y,lastpc[i].x);
                // With this line, sensor_map[0] will contain the point in
                // front of the robot
                // int ialpha = round(alpha / angular_range);

                // With this one, there is no sector just in front of the robot
                int ialpha = ceil(alpha / angular_range);
                sensor_it = sensor_map.find(ialpha);
                if (sensor_it == sensor_map.end()) {
                    sensor_map[ialpha] = r;
                } else {
                    sensor_it->second = std::min(r,sensor_it->second);
                }
            }
            // At this stage sensor_map[0] corresponds to the closest range in
            // [-30,0], sensor_map[1] corresponds to the closest range in
            // [0 : 30] degrees, etc

            // ROS_INFO("Sensor map: %.2f %.2f %.2f %.2f %.2f", sensor_map[-2],sensor_map[-1],
            //         sensor_map[0], sensor_map[1], sensor_map[2]);
            
            try {
                double min_forward = std::min(sensor_map[0],sensor_map[1]);
                desired_speed.linear.x = std::min(1.0, std::max((min_forward-safety_range)/(dont_care_range-safety_range),0.0))*max_linear_speed;
                desired_speed.angular.z = max_angular_speed*(std::min(sensor_map[1],dont_care_range)-std::min(sensor_map[0],dont_care_range))/dont_care_range;
                if (desired_speed.angular.z < -max_angular_speed) {
                    desired_speed.angular.z = -max_angular_speed;
                } 
                if (desired_speed.angular.z > max_angular_speed) {
                    desired_speed.angular.z = max_angular_speed;
                } 
                // safety to make sure we're not stuck
                if ((fabs(desired_speed.linear.x)<2e-2)) {
                    desired_speed.angular.z = 0.5;
                }
            } catch (std::exception & e) {
                ROS_ERROR("Exception in speed computation: %s",e.what());
                desired_speed.linear.x = 0;
                desired_speed.angular.z = 0;
            }
        }

    public:
        BehaviorBase() : nh("~") {
            // scanSub = nh.subscribe("scans",1,&CollisionAvoidance::pc_callback,this);
            scanSub = nh.subscribe("scans",1,&BehaviorBase::pc_callback,this);
            velPub = nh.advertise<geometry_msgs::Twist>("vel_output",1);

            nh.param("angular_range_deg",angular_range,30.); angular_range *= M_PI/180.;
            nh.param("safety_range",safety_range, 0.25);
            nh.param("dont_care_range",dont_care_range, 1.0);
            nh.param("max_linear_speed",max_linear_speed, 0.0);
            nh.param("max_angular_speed",max_angular_speed, 1.0);

        }

        void main_loop() {
            ros::Rate rate(10);
            while (ros::ok()) {
                ros::spinOnce();
                if ((ros::Time::now() - last_sensor_time).toSec() > 1.0) { 
                    desired_speed.linear.x = 0;
                    desired_speed.angular.z = 0;
                }
                velPub.publish(desired_speed);
                rate.sleep();
            }
        }
};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"behavior_base");

    BehaviorBase ca;

    ca.main_loop();
}


