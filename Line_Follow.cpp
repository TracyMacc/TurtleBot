/* Node Name: Line Follow
/* Description: Line follow Program.  
 * Subscribe Topic:  
 *    /camera/prossed
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

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int bandwidthDivider = 0;

image_transport::Publisher pub_proc;

/* when Node subscrib Image from */
void followCallback(const sensor_msgs::ImageConstPtr& msg){

}
