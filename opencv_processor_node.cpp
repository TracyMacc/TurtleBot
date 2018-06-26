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
 *    /linepoints
 */
//#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

//#include <vector>
//#include <list>

#include "ros/ros.h"
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


// callback that is executet each time an image is sent to this node
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

  Mat raw = cv_bridge::toCvShare(msg, "bgr8")->image;
  Mat hsv, hsv_mono, processed;

  // convert rgb to hsv image for color filtering

  cvtColor(raw, hsv, COLOR_BGR2HSV);
  
  // color filter and process to smooth binary
  inRange(hsv, Scalar(hue_low, sat_low, val_low), Scalar(hue_high, sat_high, val_high), hsv_mono);
  //  cvtColor(raw, mono, COLOR_BGR2GRAY);
  GaussianBlur(hsv_mono, processed, Size(9, 9), 2, 2);
  threshold(processed, processed, 0, 255, THRESH_BINARY_INV|THRESH_OTSU);

  sensor_msgs::ImagePtr msg_proc = cv_bridge::CvImage(std_msgs::Header(), "mono8", processed).toImageMsg();

  // publish processed and generated images and results
  pub_proc.publish(msg_proc);

}

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "linedetect_node");
  ros::NodeHandle n;

  // dynamic_reconfigure init
  dynamic_reconfigure::Server<opencv_processor::LinedetectConfig> lconf_server;
  dynamic_reconfigure::Server<opencv_processor::LinedetectConfig>::CallbackType f;

  f = boost::bind(&configCallback, _1, _2);
  lconf_server.setCallback(f);
  
  // publishers and subscribers

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_raw = it.subscribe("camera/image", 1, imageCallback);
  
  pub_proc = it.advertise("camera/processed", 1);

  ros::spin();
  return 0;
}
