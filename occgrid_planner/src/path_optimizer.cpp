
#include <vector>
#include <string>
#include <map>
#include <list>


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <occgrid_planner/Trajectory.h>

class PathOptimizer {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber path_sub_;
        ros::Publisher traj_pub_;
        double velocity_;
        double max_acceleration_;
        double max_braking_;

        void path_cb(const nav_msgs::PathConstPtr & msg) {
            // First pre-compute heading, velocity, rotation speed and curvilinear
            // abscissa.
            std::vector<float> omega(msg->poses.size(),0.0);
            std::vector<float> s(msg->poses.size(),0.0);
            std::vector<float> heading(msg->poses.size(),0.0);
            for (unsigned i=1;i<msg->poses.size();i++) {
                const geometry_msgs::Point& P0 = msg->poses[i-1].pose.position;
                const geometry_msgs::Point& P1 = msg->poses[i].pose.position;
                double ds = hypot(P1.y-P0.y,P1.x-P0.x);
                double dt = ds / velocity_;
                heading[i] = atan2(P1.y-P0.y,P1.x-P0.x);
                s[i] = s[i-1] + ds;
                omega[i] = remainder(heading[i]-heading[i-1], 2*M_PI) / dt;
            }
            if (heading.size()>1) {
                heading[0] = heading[1];
            }
            // Now create a trajectory message using these values.
            occgrid_planner::Trajectory output;
            output.header = msg->header;
            output.Ts.resize(msg->poses.size());
            for (unsigned i=0;i<msg->poses.size();i++) {
                output.Ts[i].header = msg->poses[i].header;
                output.Ts[i].header.stamp = output.header.stamp + ros::Duration(s[i]/velocity_);
                output.Ts[i].pose.position = msg->poses[i].pose.position;
                tf::Quaternion q = tf::createQuaternionFromRPY(0,0,heading[i]);
                tf::quaternionTFToMsg(q, output.Ts[i].pose.orientation);
                output.Ts[i].twist.linear.x = velocity_;
                output.Ts[i].twist.angular.z = omega[i];
            }
            if (output.Ts.size()>0) {
                // Set last point to zero speed, for safety.
                unsigned int j = output.Ts.size() - 1;
                output.Ts[j].twist.linear.x = 0;
                output.Ts[j].twist.angular.z = 0;
            }
            traj_pub_.publish(output);
            ROS_INFO("Optimized path into a trajectory");
        }

    public:
        PathOptimizer() : nh_("~") {
            nh_.param("velocity",velocity_,1.0);
            nh_.param("max_acceleration",max_acceleration_,1.0);
            nh_.param("max_braking",max_braking_,1.0);
            path_sub_ = nh_.subscribe<nav_msgs::Path>("path",1,&PathOptimizer::path_cb,this);
            traj_pub_ = nh_.advertise<occgrid_planner::Trajectory>("trajectory",1,true);
        }
};

int main(int argc, char * argv[]) {
    ros::init(argc,argv,"path_optimizer");
    PathOptimizer po;
    ros::spin();
}



