
#include <ros/ros.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#define INPUT_TOPIC "camera/rgb/image_color"
#define OUTPUT_TOPIC "detector/blurred"

using namespace cv;
using namespace std;

int name;

void callback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  const sensor_msgs::Image img = *msg;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
  Mat cvImage = image->image;

  stringstream ss;
  ss << "face" << name << ".jpg";
  imwrite(ss.str(), cvImage);
}

int main( int argc, char** argv)
{

  //Init ROS
  ros::init(argc, argv, "ImageRecorder");
  ros::NodeHandle n;


  //Set up image publish and subscribe
  image_transport::ImageTransport it(n);

  image_transport::Subscriber sub = 
    it.subscribe(INPUT_TOPIC,
		 1,
		 callback);

  name = 0;

  //Transfer control to ROS
  ros::spin();

  return 0;

}

