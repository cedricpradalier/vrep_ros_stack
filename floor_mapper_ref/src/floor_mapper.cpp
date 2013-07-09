#include <stdlib.h>
#include <stdio.h>

#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>





class FloorMapper {
    protected:
        ros::NodeHandle nh_;
        image_transport::Publisher ptpub_;
        image_transport::Publisher tpub_;
        image_transport::Subscriber tsub_;
        image_transport::ImageTransport it_;

        tf::TransformListener listener_;

        int floor_size_pix;
        double floor_size_meter;
        double proj_scale;
        double projected_floor_size_meter;
        std::string target_frame;

        cv::Mat floor_;
    public:
        FloorMapper() : nh_("~"), it_(nh_) {
            std::string transport = "raw";
            nh_.param("transport",transport,transport);
            // Image size to represent the environment
            nh_.param("floor_size_pix",floor_size_pix,1000);
            // Scaling of the environment, in meter
            nh_.param("floor_size_meter",floor_size_meter,50.);
            // Scaling of the image received as project floor in meter
            nh_.param("projected_floor_size_meter",projected_floor_size_meter,4.0);
            // Where to project
            nh_.param("target_frame",target_frame,std::string("/world"));

            proj_scale = projected_floor_size_meter / floor_size_meter;

            tsub_ = it_.subscribe<FloorMapper>("projection",1, &FloorMapper::callback,this,transport);
            ptpub_ = it_.advertise("p_floor",1);
            tpub_ = it_.advertise("floor",1);

            // This matrix/image will represent the floor. 0x80 (=128) is half of the
            // range of a 8-bit integer. We will represent probabilities as
            // values between 0 and 255, where 255 represents 1.0.
            floor_ = cv::Mat_<uint8_t>(floor_size_pix,floor_size_pix,0x80);

        }

        void callback(const sensor_msgs::ImageConstPtr& msg) {
            // First extract the image to a cv:Mat structure, from opencv2
            cv::Mat img(cv_bridge::toCvShare(msg,"mono8")->image);
            try{
                // Then receive the transformation between the robot body and
                // the world
                tf::StampedTransform transform;
                // Use the listener to find where we are. Check the
                // tutorials... (note: the arguments of the 2 functions below
                // need to be completed
                listener_.waitForTransform("","",ros::Time::now(),ros::Duration(1.0));
                listener_.lookupTransform("","", ros::Time::now(), transform);
                

                double proj_x = transform.getOrigin().x();
                double proj_y = transform.getOrigin().y();
                double proj_theta = -tf::getYaw(transform.getRotation());
                printf("We were at %.2f %.2f theta %.2f\n",proj_x,proj_y,proj_theta*180./M_PI);

                // Once the transformation is know, you can use it to find the
                // affine transform mapping the local floor to the global floor
                // and use cv::warpAffine to fill p_floor
                cv::Mat_<uint8_t> p_floor(floor_size_pix,floor_size_pix,0xFF);
                cv::Mat affine = (cv::Mat_<float>(2,3) 
                        << 1, 0, 0,   
                           0, 1, 0);
                cv::warpAffine(img,p_floor,affine, p_floor.size(), 
                        cv::INTER_NEAREST,cv::BORDER_CONSTANT,0xFF);

                // This published the projected floor on the p_floor topic
                cv_bridge::CvImage pbr(msg->header,"mono8",p_floor);
                ptpub_.publish(pbr.toImageMsg());

                // Now that p_floor and floor have the same size, you can use
                // the following for loop to go through all the pixels and fuse
                // the current observation with the previous one.
                for (unsigned int j=0;j<(unsigned)floor_.rows;j++) {
                    for (unsigned int i=0;i<(unsigned)floor_.cols;i++) {
                        uint8_t p = p_floor.at<uint8_t>(i,j);
                        uint8_t f = floor_.at<uint8_t>(i,j);
                        
                        // Compute the new value of pixel i,j here

                        floor_.at<uint8_t>(i,j) = f;
                    }
                }
                // Finally publish the floor estimation.
                cv_bridge::CvImage br(msg->header,"mono8",floor_);
                tpub_.publish(br.toImageMsg());
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
        }
        
};


int main(int argc, char *argv[]) {
    ros::init(argc,argv,"floor_mapper");

    FloorMapper fm;

    ros::spin();

    return 0;
}
        

