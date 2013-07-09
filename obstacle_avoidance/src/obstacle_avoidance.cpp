
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define SQR(X) ((X)*(X))

class ObstacleAvoidance {
    protected:
        ros::Subscriber scan_sub_;
        ros::Subscriber current_vel_sub_;
        ros::Subscriber command_vel_sub_;
        ros::Publisher safe_vel_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        double robot_radius_;
        double time_horizon_;
        double max_linear_velocity_;
        double max_angular_velocity_;
        double max_linear_accel_;
        double max_angular_accel_;
        double max_range_;
        double map_resolution_;
        double alpha_resolution_;
        double linear_velocity_resolution_;
        double angular_velocity_resolution_;
        double k_v_;
        double k_w_;
        std::string base_frame_;

        geometry_msgs::Twist current_velocity_;
        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        unsigned int grid_width_, n_d_, n_alpha_;
        cv::Mat_<uint8_t> og_, d_alpha_;
        cv::Mat_<float> dalpha_remap_x, dalpha_remap_y;

    public:
        static void xy_to_dalpha(double x, double y, double & d, double & alpha) {
            if (fabs(y) < 1e-3) {
                // straight line
                d = x;
                alpha = (y<0)?(-M_PI/2):(M_PI/2);
            } else {
                double h = hypot(x,y);
                double r = h*h/2*y;
                alpha = atan(r);
                if (y>=0) {
                    double gamma = atan2(x, r-y);
                    d = gamma * r;
                } else {
                    double gamma = atan2(x,y-r);
                    d = -gamma * r;
                }
            }
        }

        static void dalpha_to_xy(double d, double alpha, double & x, double & y) {
            if (fabs(fabs(alpha)-M_PI/2) < 1e-3) {
                // straight line
                x = d;
                y = 0;
            } else if (fabs(alpha) < 1e-3) {
                x = y = 0;
            } else {
                double r = tan(alpha);
                double gamma = d / r;
                x = r * sin(gamma);
                y = r * (1 - cos(gamma));
            }
        }
        
        static void test_one_conversion(double x, double y) {
            double d,alpha,x1,y1;
            xy_to_dalpha(x,y,d,alpha);
            dalpha_to_xy(d,alpha,x1,y1);
            if (hypot(x-x1,y-y1)>1e-2) {
                printf("Conversion: %.3f %.3f -> %.3f %.3f -> %.3f %.3f XXX\n",x,y,d,alpha,x1,y1);
            } else {
                printf("Conversion: %.3f %.3f -> %.3f %.3f -> %.3f %.3f\n",x,y,d,alpha,x1,y1);
            }

        }

        static void test_conversions() {
            test_one_conversion(0,0);
            test_one_conversion(1,0);
            test_one_conversion(1,1);
            test_one_conversion(0,1);
            test_one_conversion(-1,1);
            test_one_conversion(-1,0);
            test_one_conversion(-1,-1);
            test_one_conversion(0,-1);
            test_one_conversion(1,-1);
        }

    protected: // ROS Callbacks
        void command_velocity_cb(const geometry_msgs::TwistConstPtr msg) {
            geometry_msgs::Twist filtered = findClosestAcceptableVelocity(*msg);
            // ROS_INFO("Speed limiter: desired %.2f controlled %.2f",msg->linear.x,filtered.linear.x);
            safe_vel_pub_.publish(filtered);
        }

        void current_velocity_cb(const geometry_msgs::TwistStampedConstPtr msg) {
            current_velocity_ = msg->twist;
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);
            // unsigned int n = lastpc.size();
            // ROS_INFO("New point cloud: %d points",n);
            // for (unsigned int i=0;i<n;i++) {
            //     float x = lastpc[i].x;
            //     float y = lastpc[i].y;
            //     float z = lastpc[i].z;
            //     ROS_INFO("%d %.3f %.3f %.3f",i,x,y,z);
            // }
            // printf("\n\n\n");

            og_ = FREE; // reset the map
            unsigned int n = lastpc_.size();
            for (unsigned int i=0;i<n;i++) {
                float x = lastpc_[i].x;
                float y = lastpc_[i].y;
                float d = hypot(x,y);
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                cv::Point P(x / map_resolution_ + grid_width_/2., y/map_resolution_ + grid_width_/2.);
                cv::circle(og_,P, ceil(robot_radius_/map_resolution_ + 1), cv::Scalar(OCCUPIED), -1); // filled circle
            }
            // Marked the robot own space as FREE, whatever the point cloud
            // is saying. This means that rotation on the spot are always
            // possible
            cv::Point myself(grid_width_/2,grid_width_/2);
            cv::circle(og_,myself, ceil(robot_radius_/map_resolution_ + 1), cv::Scalar(FREE), -1); // filled circle
            cv::imshow( "OccGrid", og_ );
            //
            // First convert the obstacle into the ego-kinematic space
            // (d,alpha), where d is the distance on the arc of circle, and
            // alpha = atan2(v,w)
            cv::remap(og_,d_alpha_,dalpha_remap_x,dalpha_remap_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, UNKNOWN );
            // Now we need to go through all the d_alpha combination. If we hit
            // an obstacle at a given value of d, any further distance is also
            // going to hit it.
            for (unsigned int i=0;i<n_alpha_;i++) {
                for (int j=n_d_/2;j<(signed)n_d_;j++) {
                    if (d_alpha_(j,i) != FREE) {
                        for (;j<(signed)n_d_;j++) d_alpha_(j,i) = OCCUPIED;
                    }
                }
                for (int j=n_d_/2;j>=0;j--) {
                    if (d_alpha_(j,i) != FREE) {
                        for (;j>=0;j--) d_alpha_(j,i) = OCCUPIED;
                    }
                }
            }
            cv::imshow( "DAlpha", d_alpha_ );

        }

        geometry_msgs::Twist findClosestAcceptableVelocity(const geometry_msgs::Twist & desired) {
            geometry_msgs::Twist res = desired;
            // Now build Vs inter Vd
            double min_v = std::max(-max_linear_velocity_, current_velocity_.linear.x - max_linear_accel_*time_horizon_);
            double max_v = std::min(max_linear_velocity_, current_velocity_.linear.x + max_linear_accel_*time_horizon_);
            double min_w = std::max(-max_angular_velocity_, current_velocity_.angular.z - max_angular_accel_*time_horizon_);
            double max_w = std::min(max_angular_velocity_, current_velocity_.angular.z + max_angular_accel_*time_horizon_);
            unsigned int n_v = ceil((max_v-min_v)/linear_velocity_resolution_+1); 
            unsigned int n_w = ceil((max_w-min_w)/angular_velocity_resolution_+1); 
            // printf("Vds: v %d [%.2f %.2f] w %d [%.2f %.2f]\n",n_v, min_v, max_v, n_w, min_w, max_w);
            cv::Mat_<uint8_t> Vds(n_v,n_w,FREE); // Vs inter Vd
            cv::Mat_<uint8_t> scores(n_v,n_w,(uint8_t)OCCUPIED); // Vs inter Vd

            // Now build Va (inside Vs inter Vd) by iterating over the local
            // map, and find the most appropriate speed
            // FILE * fp = fopen("/tmp/V.txt","w");
            double best_score = 0;
            double best_v = 0, best_w = 0;
            for (unsigned int j=0;j<n_v;j++) {
                double v = min_v + j*linear_velocity_resolution_;
                // double th = fabs(v/max_linear_accel_);
                double d = v * time_horizon_ ;
                int i_d = round((d + max_range_) / map_resolution_);
                if (i_d < 0) i_d =0;
                if (i_d >= (signed)n_d_) i_d = n_d_-1;
                for (unsigned int i=0;i<n_w;i++) {
                    double w = min_w + i*angular_velocity_resolution_;
                    double alpha = atan2(v,w);
                    if (alpha < 0) alpha += 2*M_PI;
                    if (alpha >= M_PI) alpha -= M_PI;
                    int i_alpha = (int)(round(alpha / alpha_resolution_)) % n_alpha_;
                    uint8_t value;
                    if (fabs(v)<linear_velocity_resolution_) { 
                        // Force rotation on the spot to be always possible
                        // They should be, but alpha is not super well defined
                        // for v = 0
                        value = FREE;
                    } else {
                        value = d_alpha_(i_d, i_alpha);
                    }
                    Vds(j,i) = value;
                    double score = 0;
                    if (value == FREE) {
                        score = exp(-(k_v_ * SQR(v - desired.linear.x)
                                    + k_w_ * SQR(w - desired.angular.z)));
                        scores(j,i) = 0x80 + score*0x7F;
                        if (score > best_score) {
                            best_v = v;
                            best_w = w;
                            best_score = score;
                        }
                    }
                    // fprintf(fp,"Control %.2f %.2f, d %.2f alpha %.2f (r %.2f) value %d score %.2f\n",
                    //         v,w,d,alpha,tan(alpha), value, score);
                }
            }
            // fclose(fp);
            cv::resize(scores,scores,cv::Size(200,200));
            cv::imshow("Vr",scores);

            res.linear.x = best_v;
            res.angular.z = best_w;

            return res;
        }

    public:
        ObstacleAvoidance() : nh_("~") {
            alpha_resolution_ = 3. * M_PI/180.;

            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("max_linear_velocity",max_linear_velocity_,0.5);
            nh_.param("max_angular_velocity",max_angular_velocity_,1.0);
            nh_.param("max_linear_accel",max_linear_accel_,0.5);
            nh_.param("max_angular_accel",max_angular_accel_,0.5);
            nh_.param("map_resolution",map_resolution_,0.05);
            nh_.param("linear_velocity_resolution",linear_velocity_resolution_,0.05);
            nh_.param("angular_velocity_resolution",angular_velocity_resolution_,0.05);
            nh_.param("robot_radius",robot_radius_,0.1);
            nh_.param("time_horizon",time_horizon_,0.1);
            nh_.param("k_v",k_v_,1.0);
            nh_.param("k_w",k_w_,1.0);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&ObstacleAvoidance::pc_callback,this);
            // scan_sub_ = nh_.subscribe("/vrep/hokuyoSensor",1,&ObstacleAvoidance::pc_callback,this);
            current_vel_sub_ = nh_.subscribe("current_velocity",1,&ObstacleAvoidance::current_velocity_cb,this);
            command_vel_sub_ = nh_.subscribe("command_velocity",1,&ObstacleAvoidance::command_velocity_cb,this);
            safe_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("output_velocity",1);

            grid_width_ = 2*max_range_ / map_resolution_ + 1;
            og_ = cv::Mat_<uint8_t>(grid_width_,grid_width_,FREE);
            n_d_ = ceil(2*max_range_/map_resolution_); 
            n_alpha_ = ceil(M_PI / alpha_resolution_); 
            d_alpha_ = cv::Mat_<uint8_t>(n_d_,n_alpha_,FREE); // Vs inter Vd


            // Now prepare the remapping to convert obstacles between circular
            // coordinates and cartesian coorsinates
            unsigned int w = 2*max_range_ / map_resolution_ + 1;
            unsigned int nd = ceil(2*max_range_/map_resolution_); 
            unsigned int nalpha = ceil(M_PI / alpha_resolution_); 
            dalpha_remap_x = cv::Mat_<float>(nd,nalpha);
            dalpha_remap_y = cv::Mat_<float>(nd,nalpha);
            FILE * fp = fopen("/tmp/maps.txt","w");
            for (unsigned int j=0;j<nd;j++) {
                double d = (j - nd/2.) * map_resolution_;
                for (unsigned int i=0;i<nalpha;i++) {
                    double alpha = i * alpha_resolution_;
                    double x=0, y=0;
                    dalpha_to_xy(d,alpha,x,y);
                    fprintf(fp,"%d %d %.3f %.3f -> %.3f %.3f\n",j,i,d,alpha,x,y);
                    dalpha_remap_x(j,i) = x / map_resolution_ + w/2.;
                    dalpha_remap_y(j,i) = y / map_resolution_ + w/2.;
                }
            }
            // cv::imshow("DAlphaX",dalpha_remap_x);
            // cv::imshow("DAlphaY",dalpha_remap_y);
            fclose(fp);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"obstacle_avoidance");
    cv::namedWindow( "OccGrid", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "DAlpha", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "Vr", CV_WINDOW_AUTOSIZE );
    // cv::namedWindow( "DAlphaX", CV_WINDOW_AUTOSIZE );
    // cv::namedWindow( "DAlphaY", CV_WINDOW_AUTOSIZE );

    ObstacleAvoidance::test_conversions();
    ObstacleAvoidance ca;

    while (ros::ok()) {
        ros::spinOnce();
        cv::waitKey(50);
    }
    // TODO: implement a security layer
}


