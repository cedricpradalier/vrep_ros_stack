
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


class CollisionAvoidance {
    protected:
        ros::Subscriber scanSub;
        ros::Subscriber velSub;
        ros::Publisher velPub;

        ros::NodeHandle nh;
        double safety_diameter;
        double time_horizon;

        pcl::PointCloud<pcl::PointXYZ> lastpc;

        void velocity_filter(const geometry_msgs::TwistConstPtr msg) {
            geometry_msgs::Twist filtered = findClosestAcceptableVelocity(*msg);
            velPub.publish(filtered);
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::fromROSMsg(*msg, lastpc);
            // unsigned int n = lastpc.size();
            // ROS_INFO("New point cloud: %d points",n);
            // for (unsigned int i=0;i<n;i++) {
            //     float x = lastpc[i].x;
            //     float y = lastpc[i].y;
            //     float z = lastpc[i].z;
            //     ROS_INFO("%d %.3f %.3f %.3f",i,x,y,z);
            // }
            // printf("\n\n\n");
        }

        geometry_msgs::Twist findClosestAcceptableVelocity(const geometry_msgs::Twist & desired) {
            geometry_msgs::Twist res = desired;
            unsigned int n = lastpc.size();
            double multiplier = 1;
            for (unsigned int i=0;i<n;i++) {
                float x = lastpc[i].x;
                float y = lastpc[i].y;

                // float z = *(float*)(&lastpc.data[lastpc.point_step*i + lastpc.fields[2].offset]);
                if (hypot(x,y) < 1e-2) {
                    // bogus point, the laser did not return
                    continue;
                }

                if (fabs(y) > safety_diameter) {
                    // too far on the side
                    continue;
                }
                if (x*res.linear.x < 0) {
                    // not going towards it
                    continue;
                }

                if (fabs(x) < safety_diameter) {
                    ROS_INFO("Too close! %.2f %.2f",x,y);
                    multiplier = 0;
                } else if (fabs(x) > 3 * safety_diameter) {
                    continue;
                } else {
                    multiplier = std::min(multiplier,desired.linear.x*(fabs(x)-safety_diameter)/(2*safety_diameter));
                }
                res.linear.x = std::min(multiplier, desired.linear.x);
            }
            ROS_INFO("Speed limiter: desired %.2f controlled %.2f",desired.linear.x,res.linear.x);

            return res;
        }

    public:
        CollisionAvoidance() : nh("~") {
            // scanSub = nh.subscribe("scans",1,&CollisionAvoidance::pc_callback,this);
            scanSub = nh.subscribe("/vrep/hokuyoSensor",1,&CollisionAvoidance::pc_callback,this);
            velSub = nh.subscribe("vel_input",1,&CollisionAvoidance::velocity_filter,this);
            velPub = nh.advertise<geometry_msgs::Twist>("vel_output",1);

            safety_diameter = 0.2; // [m]
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"collision_avoidance");

    CollisionAvoidance ca;

    ros::spin();
    // TODO: implement a security layer
}


