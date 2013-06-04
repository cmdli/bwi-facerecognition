
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

using namespace std;
using namespace cv;

HOGDescriptor detector;

image_transport::Publisher publisher;

void callback(const sensor_msgs::ImageConstPtr &imgptr)
{
  //Load image into OpenCV
  const sensor_msgs::Image img = *imgptr;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(img);
  cv::Mat cvImage = image->image;
  cv::Mat cvGray;

  //Convert image to grayscale
  cvtColor(cvImage, cvGray, CV_RGB2GRAY);
  cv::Mat cvOutput(cvGray);

  //Find people in image
  vector<Rect> personRects;
  detector.detectMultiScale(cvGray, personRects);

  //Draw boxes around each person
  for(int i = 0; i < personRects.size(); i++) {

    //If the person is too small, exclude it
    if (personRects[i].width > 200)
      continue;

    rectangle( cvImage, personRects[i], Scalar(255,255,255));

  }

  image->image = cvImage;

  //Publish image
  publisher.publish(image->toImageMsg());


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "person_detector");
  ros::NodeHandle n;

  //Set up topics to subscribe and publish to
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = 
    it.subscribe("rgb_input",
		 1,
		 callback);
  publisher = it.advertise("output",10);

  detector.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

  ROS_INFO("Transfer control to ROS");
  ros::spin();

  return 0;


}
