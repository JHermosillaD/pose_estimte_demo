#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <pose_estimate_demo/ImageBoundingBox.h>

using namespace std;
using namespace cv;

string image_topic;
string hog_bbox_topic;
string haar_bbox_topic;

class ShowDemo {

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber hog_sub_;
  ros::Subscriber haar_sub_;
  image_transport::Publisher image_pub_;
  bool validFace = false;
  bool validBody = false;
  int haar_u_px, haar_v_px, haar_width, haar_height;
  int hog_u_px, hog_v_px, hog_width, hog_height;

public:
  ShowDemo()
  : it_(nh_) {
    image_sub_ = it_.subscribe(image_topic, 1, &ShowDemo::cameraCallback, this);
    hog_sub_ = nh_.subscribe(hog_bbox_topic, 1000, &ShowDemo::hogCallback, this);
    haar_sub_ = nh_.subscribe(haar_bbox_topic, 1000, &ShowDemo::haarCallback, this);
    image_pub_ = it_.advertise("/pedestrian/image", 1);
  }

  void hogCallback(const pose_estimate_demo::ImageBoundingBox& bbox_msg) {

    if (isnan(bbox_msg.cornerPoints[0].u) == false) {
      hog_u_px = bbox_msg.cornerPoints[0].u;
      hog_v_px = bbox_msg.cornerPoints[0].v;
      hog_width = bbox_msg.width;
      hog_height = bbox_msg.height;
      if (hog_u_px!=0 && hog_v_px!=0){
	      validBody = true;
      }
      else
	      validBody = false;
    }
  }

  void haarCallback(const pose_estimate_demo::ImageBoundingBox& bbox_msg) {

    if (isnan(bbox_msg.cornerPoints[0].u) == false) {
      haar_u_px = bbox_msg.cornerPoints[0].u + bbox_msg.width/2;
      haar_v_px = bbox_msg.cornerPoints[0].v + bbox_msg.height/2;
      haar_width = bbox_msg.width;
      haar_height = bbox_msg.height;
      if (haar_u_px!=0 && haar_v_px!=0){
	      validFace = true;
      }
      else
	      validFace = false;
    }
  }

  void cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
    
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (validFace = true) {
      Point center(haar_u_px, haar_v_px);
      ellipse(cv_ptr->image,center,Size(haar_width*0.5,haar_height*0.5), 0, 0, 360, Scalar(0,0,255), 4, 8, 0);
    }
    if (validBody = true) {
        Point left_corner(hog_u_px, hog_v_px);
        Point right_corner(hog_u_px + hog_width, hog_v_px + hog_height);
        rectangle(cv_ptr->image,left_corner,right_corner,Scalar(255,0,0), 2, LINE_8);

    }
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle nh;
  nh.getParam("/camera_topic", image_topic);
  nh.getParam("/hog_bbox_topic", hog_bbox_topic);
  nh.getParam("/haar_bbox_topic", haar_bbox_topic);
  ShowDemo ic;
  ros::spin ();
  return 0;
}
