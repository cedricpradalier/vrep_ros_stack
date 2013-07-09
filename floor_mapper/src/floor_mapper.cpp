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
        image_transport::Publisher ptpub_; // mostly for debug
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
            nh_.param("floor_size_pix",floor_size_pix,1000);
            nh_.param("floor_size_meter",floor_size_meter,50.);
            nh_.param("projected_floor_size_meter",projected_floor_size_meter,4.0);
            nh_.param("target_frame",target_frame,std::string("/world"));
            proj_scale = projected_floor_size_meter / floor_size_meter;

            tsub_ = it_.subscribe<FloorMapper>("projection",1, &FloorMapper::callback,this,transport);
            ptpub_ = it_.advertise("p_floor",1);
            tpub_ = it_.advertise("floor",1);

            floor_ = cv::Mat_<uint8_t>(floor_size_pix,floor_size_pix,0x80);

        }

        void callback(const sensor_msgs::ImageConstPtr& msg) {
            cv::Mat img(cv_bridge::toCvShare(msg,"mono8")->image);
            tf::StampedTransform transform;
            try{
                listener_.waitForTransform(target_frame,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
                listener_.lookupTransform(target_frame,msg->header.frame_id, 
                        msg->header.stamp, transform);
                double proj_x = transform.getOrigin().x();
                double proj_y = transform.getOrigin().y();
                double proj_theta = -tf::getYaw(transform.getRotation());
                printf("Image at %.2f %.2f theta %.2f\n",proj_x,proj_y,proj_theta*180./M_PI);

                cv::Mat_<uint8_t> p_floor(floor_size_pix,floor_size_pix,0xFF);
                double p_scale = projected_floor_size_meter / img.cols;
                cv::Mat p_scale_translate = (cv::Mat_<float>(3,3) 
                        << p_scale, 0,          0,   
                           0,       p_scale, -projected_floor_size_meter/2, 0,0,1);
                cv::Mat rotate = (cv::Mat_<float>(3,3) 
                        << cos(proj_theta), -sin(proj_theta), 0,   
                           sin(proj_theta),  cos(proj_theta), 0, 0,0,1);
                double scale = floor_size_pix / floor_size_meter;
                cv::Mat scale_translate = (cv::Mat_<float>(3,3) 
                        << scale, 0,     scale*proj_x + floor_.cols/2.,
                           0,     scale, -scale*proj_y + floor_.rows/2., 0,0,1);
                cv::Mat affine = scale_translate * rotate * p_scale_translate;
                cv::warpAffine(img,p_floor,affine(cv::Range(0,2),cv::Range::all()),
                        p_floor.size(), cv::INTER_LINEAR,cv::BORDER_CONSTANT,0xFF);

                cv_bridge::CvImage pbr(msg->header,"mono8",p_floor);
                ptpub_.publish(pbr.toImageMsg());

                for (unsigned int j=0;j<(unsigned)floor_.rows;j++) {
                    for (unsigned int i=0;i<(unsigned)floor_.cols;i++) {
                        uint8_t o = p_floor.at<uint8_t>(i,j);
                        if (o == 0xFF) continue;

                        float z = o / 255.;
                        float c = floor_.at<uint8_t>(i,j)/255.;

                        floor_.at<uint8_t>(i,j) = (uint8_t)((0.9*c + 0.1*z)*255);
                    }
                }
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
        

