
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include <ar_recog/Tag.h>
#include <ar_recog/Tags.h>
#include <ar_recog/CalibrateDistance.h>

#include <math.h>

#include <AR/ar.h>
#include <AR/video.h>


extern "C" { 
#include "object.h" 
}

#include <iostream>

#define THRESH 100

char cparam_name[65536];
ARParam cparam;

char model_name[65536];
ObjectData_T *object;
int objectnum;

bool using_rectified_images;

void init(int width, int height) {
  ARParam wparam;

  if (arParamLoad(cparam_name, 1, &wparam) < 0) {
    std::cerr << "Problem loading AR camera parameters." << std::endl;
    exit(-1);
  }
	
  for (int i = 0; i < 4; i++) {
    std::cout << wparam.dist_factor[i] << "\t";
  }
  std::cout << std::endl;

  arParamChangeSize(&wparam, width, height, &cparam);
  arInitCparam(&cparam);

  if ((object = read_ObjData(model_name, &objectnum)) == NULL) {
    std::cerr << "Problem reading patterns." << std::endl;
    exit(-1);
  }
}

int count;
double haov, vaov;
ros::Publisher tags_pub;
image_transport::Publisher ar_img; //used by the handling functions
int last_reported_distance; // Saved by the publisher for use by the calibration service

void handleImage(const sensor_msgs::ImageConstPtr& in, const sensor_msgs::CameraInfoConstPtr& camera_info) {

  sensor_msgs::CvBridge bridge;
  IplImage* img = bridge.imgMsgToCv(in, "rgb8");

  if (img->width != arImXsize || img->height != arImYsize) init(img->width, img->height);

  ARUint8 *data = (ARUint8*) malloc(sizeof(ARUint8)*img->width*img->height*3);
  int i = 0;
  for (int y = 0; y < img->height; y++) {
    for (int x = 0; x < img->width; x++) {
      for (int j = 0; j < 3; j++) {
	data[i++] = img->imageData[y*(img->widthStep/sizeof(uchar))+x*3+j];
      }
    }
  }

  ARMarkerInfo *marker_info;
  int marker_num;
  if(arDetectMarker(data, THRESH, &marker_info, &marker_num) < 0) {
    std::cerr << "Problem running detect marker." << std::endl;
    exit(-1);
  }

  for(int i = 0; i < marker_num; i++) {
    const int width = 3;
    cvLine(img,cvPoint(marker_info[i].vertex[0][0],marker_info[i].vertex[0][1]),
	   cvPoint(marker_info[i].vertex[1][0],marker_info[i].vertex[1][1])
	   ,CV_RGB(0,255,0),width);
    cvLine(img,cvPoint(marker_info[i].vertex[1][0],marker_info[i].vertex[1][1]),
	   cvPoint(marker_info[i].vertex[2][0],marker_info[i].vertex[2][1])
	   ,CV_RGB(0,255,0),width);
    cvLine(img,cvPoint(marker_info[i].vertex[2][0],marker_info[i].vertex[2][1]),
	   cvPoint(marker_info[i].vertex[3][0],marker_info[i].vertex[3][1])
	   ,CV_RGB(0,255,0),width);
    cvLine(img,cvPoint(marker_info[i].vertex[3][0],marker_info[i].vertex[3][1]),
	   cvPoint(marker_info[i].vertex[0][0],marker_info[i].vertex[0][1])
	   ,CV_RGB(0,255,0),width);
  }

  if (using_rectified_images) {
    // Potential non-portability issue here... P uses ROS float64, but
    // I'm just using doubles for now.

    // These numbers are not always accurate after a calibration.  It
    // may be that our checkerboard calibrators are insufficient to
    // nail down the parameters of cheap webcams, or that the
    // calibration routines within opencv are not the greatest.
    // Uncalibrated raw image data seems to work pretty well, though,
    // once you've measured a known distance.

    double y_foc = camera_info->P[5];
    double y_princ = camera_info->P[6];
    double x_foc = camera_info->P[0];
    double x_princ = camera_info->P[2];

    // Camera angle of view
    haov = 2.0 * atan(x_princ / x_foc);
    vaov = 2.0 * atan(y_princ / y_foc);
  }
  else {
    vaov = haov * ((double)img->height / (double)img->width);
  } 

  ar_recog::Tags tags;	
  tags.image_width = img->width;
  tags.image_height = img->height;
  tags.angle_of_view = haov;

  for( int i = 0; i < objectnum; i++ ) {
    int k = -1;
    for( int j = 0; j < marker_num; j++ ) {
      if( object[i].id == marker_info[j].id) {
	if( k == -1 ) 
	  k = j;
	else /* make sure you have the best pattern (highest confidence factor) */
	  if( marker_info[k].cf < marker_info[j].cf ) k = j;
      }
    }

    if( k == -1 ) {
      object[i].visible = 0;
      continue;
    }
		
    /* calculate the transform for each marker */
    if( object[i].visible == 0 ) {
      arGetTransMat(&marker_info[k], 
		    object[i].marker_center, 
		    object[i].marker_width, 
		    object[i].trans);
    } else {
      arGetTransMatCont(&marker_info[k], 
			object[i].trans, 
			object[i].marker_center, 
			object[i].marker_width, 
			object[i].trans);
    }

    object[i].visible = 1;

    int x, y;
    double diameter;
    double xRot, yRot, zRot;
    //double xMetric, yMetric, zMetric;

    x = marker_info[k].pos[0];
    y = marker_info[k].pos[1];
    //diameter = (int) ((double) .5 + (double) sqrt(marker_info[k].area));

    xRot = asin(object[i].trans[1][2]);

    yRot = -asin(object[i].trans[0][2]);

    zRot = asin(object[i].trans[1][0]);

    // This relative x and y stuff corrects for the distortion that
    // comes from an AR tag moving out of the fovea.  When the tag is
    // at the edge of the image, it appears rotated, even when it is
    // parallel to the image plane.  This is not the behavior we want,
    // and leads to inaccurate distance measurement, for one thing.
    double x_delta = (double)(2*y-img->height)/(double)img->height;
    double y_delta = (double)(2*x-img->width)/(double)img->width;
    double rel_theta = x_delta * haov / 2.0;
    double rel_phi = y_delta * vaov / 2.0;

    // xRot is rotation around the horizontal axis, which means
    // changes in apparent _vertical_ size, and vice versa with yRot.
    // Which is why these look backwards.
    xRot -= y_delta * rel_phi;
    yRot -= x_delta * rel_theta;

    double unrotated_area = marker_info[k].area / (cos(xRot)*cos(yRot));
    diameter = sqrt(unrotated_area);

    double theta_object = haov * (diameter / (double)img->width);
    double phi_object = vaov * (diameter / (double)img->height);
    // Whichever measurement is closer to the center of the image is likelier to be more accurate.
    int dist = object[i].marker_width / tan(x_delta < y_delta ? theta_object : phi_object);

    //For the next release!  Not yet tested or debugged...
    //xMetric = dist * sin(rel_theta) * cos(rel_phi);
    //yMetric = dist * sin(rel_theta) * sin(rel_phi);
    //zMetric = dist * cos(rel_theta);
    
    ar_recog::Tag tag;
    tag.id = i;
    tag.cf = marker_info[k].cf; 
    tag.x = x;
    tag.y = y;
    tag.diameter = diameter;
    tag.distance = dist;
    tag.xRot = xRot;
    tag.yRot = yRot;
    tag.zRot = zRot;
    //tag.xMetric = xMetric;
    //tag.yMetric = yMetric;
    //tag.zMetric = zMetric;
    int b = marker_info[k].dir;
    int idx[4];
    //rotate based on direction
    switch (b) {
    case 0:
      idx[0] = 0;
      idx[1] = 1;
      idx[2] = 2;
      idx[3] = 3;
      break;
    case 3:
      idx[0] = 1;
      idx[1] = 2;
      idx[2] = 3;
      idx[3] = 0;
      break;
    case 2:
      idx[0] = 2;
      idx[1] = 3;
      idx[2] = 0;
      idx[3] = 1;
      break;
    default: //1
      idx[0] = 3;
      idx[1] = 0;
      idx[2] = 1;
      idx[3] = 2;
      break;
    }

    tag.cwCorners[0] = marker_info[k].vertex[idx[0]][0];
    tag.cwCorners[1] = marker_info[k].vertex[idx[0]][1]; //upper left
    tag.cwCorners[2] = marker_info[k].vertex[idx[1]][0];
    tag.cwCorners[3] = marker_info[k].vertex[idx[1]][1]; //upper right
    tag.cwCorners[4] = marker_info[k].vertex[idx[2]][0];
    tag.cwCorners[5] = marker_info[k].vertex[idx[2]][1]; //lower right
    tag.cwCorners[6] = marker_info[k].vertex[idx[3]][0];
    tag.cwCorners[7] = marker_info[k].vertex[idx[3]][1]; //lower left

    tags.tags.push_back(tag);

    last_reported_distance = dist;

  }

  tags.tag_count = tags.tags.size();

  sensor_msgs::ImagePtr out = sensor_msgs::CvBridge::cvToImgMsg(img, "rgb8");
  ar_img.publish(out);

  tags_pub.publish(tags);

  free(data);
}

void handleRawImage(const sensor_msgs::ImageConstPtr& in) {
	if (using_rectified_images) return;
	const sensor_msgs::CameraInfoConstPtr ign;
	handleImage(in,ign);
}

bool calibrateDistance(ar_recog::CalibrateDistance::Request &req,
		       ar_recog::CalibrateDistance::Response &resp) {

  using_rectified_images = false;
  haov = haov*((double)last_reported_distance / (double)req.dist);
  resp.aov = haov;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  std::string tmp;
  nh.param<std::string>("camera_para", tmp, "camera_para.dat");
  strcpy(cparam_name, tmp.c_str());
  nh.param<std::string>("model_name", tmp, "object_data");
  strcpy(model_name, tmp.c_str());

  if (nh.getParam("aov", haov)) {
    using_rectified_images = false;
  //nh.param<double>("aov", theta, 0.67018834495304758);
    std::cout << "aov: " << haov << std::endl;
  }
  else {
    using_rectified_images = true;
  }

  tags_pub = nh.advertise<ar_recog::Tags>("tags",1);

  ros::ServiceServer calibrate_distance = nh.advertiseService("ar/calibrate_distance", calibrateDistance);

  image_transport::ImageTransport it(nh);
  ar_img = it.advertise("ar/image", 1);

  arImXsize = arImYsize = 0;
  count = 0;

  image_transport::CameraSubscriber sub = it.subscribeCamera("image", 1, handleImage);
  image_transport::Subscriber subRaw = it.subscribe("image", 1, handleRawImage);
  ros::spin();
}
