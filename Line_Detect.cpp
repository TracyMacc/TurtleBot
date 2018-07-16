
/**
 * ROS-node for the linedetection.
 *
 * Used Topics:
 *    /camera/image
 *
 * Advertised Topics:
 *    /camera/processed
 *    /camera/processed_lowbw
 *    /camera/overlay
 *    /camera/overlay_lowbw
 *    /detect/Lane
 */
//#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

//#include <vector>
//#include <list>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include "opencv_processor/LinedetectConfig.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int bandwidthDivider = 0;

int hue_low;
int hue_high;
int sat_low;
int sat_high;
int val_low;
int val_high;

float centerX;
int centerY;

image_transport::Publisher pub_proc;

// callback from dynamic_reconfigure
void configCallback(opencv_processor::LinedetectConfig &config, uint32_t level) {

  hue_low = config.hue_min;
  hue_high = config.hue_max;
  sat_low = config.saturation_min;
  sat_high = config.saturation_max;
  val_low = config.value_min;
  val_high = config.value_max;
}

/***************************************************************************************************************************/

// callback that is executet each time an image is sent to this node
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

  Mat raw = cv_bridge::toCvShare(msg, "rgb8")->image; // Image 640*480
  Mat hsv, hsv_mono, processed, image;

  // convert rgb to hsv image for color filtering

  cvtColor(raw, hsv, COLOR_BGR2HSV);
  
  hsv = hsv(Rect(170, 330, 300, 150));
  
  // color filter and process to smooth binary
  inRange(hsv, Scalar(hue_low, sat_low, val_low), Scalar(hue_high, sat_high, val_high), hsv_mono);
  GaussianBlur(hsv_mono, processed, Size(9, 9), 2, 2);
  threshold(processed, processed, 100, 255, THRESH_BINARY_INV|THRESH_OTSU);
  cv::bitwise_not(processed,processed);
  cv::bitwise_and(raw, raw, image);
    
  Moments mu;
  mu = moments(processed, false);

  centerX = (mu.m10/mu.m00 - 150)+320;
  float centerY = mu.m01/mu.m00;
  
  int pHeight = processed.size().height; 
  int pWidth = processed.size().width;
 
  int middleX = (pWidth/2 - 150) + 320; //Get X coordenate of the Img middle point
  int middleY = pHeight/2 + 330; //Get Y coordenate of the Img middle point

  Point2f center(centerX, middleY); // point in center (x only)
  Point2f middleImg(middleX, middleY); // Img middle point
  
  cv::line(image, center, middleImg, Scalar(0, 0, 255), 2, 4, 0);
  cv::circle(image, middleImg, 5, Scalar(0, 0, 0), -1);
  cv::circle(image, center, 5, Scalar(255, 0, 0), -1);  
  
  vector<vector<cv::Point>> contours;
  cv::findContours(processed, contours, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, Point(170,330));
  cv::drawContours(image, contours, -1,Scalar(0,255,0),2);  

  sensor_msgs::ImagePtr msg_proc = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image).toImageMsg();
  
  pub_proc.publish(msg_proc);

}

/******************************************************************************************************************/
/* Main Program
 *
 */
int main(int argc, char** argv) {

  ros::init(argc, argv, "linedetect_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  // dynamic_reconfigure init
  dynamic_reconfigure::Server<opencv_processor::LinedetectConfig> lconf_server;
  dynamic_reconfigure::Server<opencv_processor::LinedetectConfig>::CallbackType f;

  f = boost::bind(&configCallback, _1, _2);
  lconf_server.setCallback(f);

  // publishers and subscribers

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_raw = it.subscribe("camera/image", 1, imageCallback);

  pub_proc = it.advertise("camera/processed", 1);

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("detect/Lane", 1000);

  while (ros::ok()){

      std_msgs::Int32 s;
      std_msgs::Int32 y;

      s.data = centerX;

      chatter_pub.publish(s);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

