

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <unistd.h>

image_transport::Publisher publisher;

using namespace std;
using namespace cv;

int hasOld = 0;
Mat oldImage;
Mat oldImage2;

void callback(const sensor_msgs::ImageConstPtr &imgptr)
{

  //Load image into OpenCV
  const sensor_msgs::Image img = *imgptr;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(img);
  cv::Mat cvImage = image->image;
  cvtColor(cvImage, cvImage, CV_RGB2GRAY);

  if(hasOld < 1) {
    oldImage = cvImage;
    hasOld++;
    return;
  }

  Mat diff;
  absdiff(cvImage, oldImage, diff);
  Scalar total;
  total = sum(diff);
  stringstream ss;
  ss << "Diff: " << sum;
  ROS_INFO(ss.str().c_str());
  
  oldImage = cvImage;

  Mat kernel = getStructuringElement(MORPH_ELLIPSE,
				     Size(7,7));


  //threshold(diff, diff, 20, 255, THRESH_TOZERO);

  for(int i = 0; i < 15; i++) {

    dilate(diff, diff, kernel);

  }

  threshold(diff, diff, 50, 255, THRESH_BINARY);

  SimpleBlobDetector detector;
  vector<KeyPoint> points;
  detector.detect(diff, points);

  for(int i = 0; i < points.size(); i++) {
    circle(diff, points[i].pt, (int)points[i].size, Scalar(255, 0, 255));
  }

  image->image = diff;
  image->encoding = "mono8";

  //Publish image
  publisher.publish(image->toImageMsg());

}



int main( int argc, char** argv)
{
  //Init ROS
  ros::init(argc, argv, "face_detector");
  ros::NodeHandle n;

  //Set up topics to subscribe and publish to
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = 
    it.subscribe("rgb_input",
		 1,
		 callback);
  publisher = it.advertise("output",10);

  ROS_INFO("Transfer control to ROS");
  ros::spin();

  return 0;

}

