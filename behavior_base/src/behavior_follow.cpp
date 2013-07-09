
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
        geometry_msgs::Twist desired_speed;

        double desired_range;
        double max_linear_speed;
        double max_angular_speed;
        double k_r;
        double k_alpha;


        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> lastpc;
            pcl::fromROSMsg(*msg, lastpc);
            last_sensor_time = ros::Time::now();
            double rmin = 100, alpha_min = 0;
            // Convert the pointcloud into a discrete set of range measurement
            // separated by ANGULAR_RANGE
            unsigned int n = lastpc.size();
            for (unsigned int i=0;i<n;i++) {
                double r = hypot(lastpc[i].y,lastpc[i].x);
                double alpha = atan2(lastpc[i].y,lastpc[i].x);
                if (r < 1e-2) {
                    // bogus point, the laser did not return
                    continue;
                }
                if (r < rmin) {
                    rmin = r;
                    alpha_min = alpha;
                }
            }
            // ROS_INFO("Sensor map: %.2f %.2f %.2f %.2f %.2f", sensor_map[-2],sensor_map[-1],
            //         sensor_map[0], sensor_map[1], sensor_map[2]);
            
            try {
                // Use the value stored in sensor_map to implement the desired
                // behaviors
                desired_speed.linear.x = k_r*(rmin - desired_range);
                if (desired_speed.linear.x > max_linear_speed) {
                    desired_speed.linear.x = max_linear_speed;
                }
                if (desired_speed.linear.x <-max_linear_speed) {
                    desired_speed.linear.x = -max_linear_speed;
                }
                double sv = (desired_speed.linear.x>=0)?+1:-1;
                desired_speed.angular.z = k_alpha * alpha_min;
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

            nh.param("max_linear_speed",max_linear_speed, 0.0);
            nh.param("max_angular_speed",max_angular_speed, 1.0);
            nh.param("desired_range",desired_range, 1.0);
            nh.param("k_r",k_r, 1.0);
            nh.param("k_alpha",k_alpha, 1.0);

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


