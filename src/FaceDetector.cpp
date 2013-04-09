
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include <iostream>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>


image_transport::Publisher publisher;

using namespace std;
using namespace cv;

CascadeClassifier faces;

void callback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  const sensor_msgs::Image img = *msg;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
  cv::Mat cvImage = image->image;
  cv::Mat cvOutput; 

  cvtColor(cvImage, cvOutput, CV_RGB2GRAY);

  vector<Rect> faceRects;

  faces.detectMultiScale(cvOutput, faceRects);

  for(int i = 0; i < faceRects.size(); i++) {
    rectangle( cvOutput, faceRects[i], Scalar(255,0,0));
    float x1 = faceRects[i].x;
    float x2 = x1 + faceRects[i].width;
    float y1 = faceRects[i].y;
    float y2 = y1 + faceRects[i].height;
  }
  
  image->image = cvOutput;
  image->encoding = "mono8";

  //Publish image
  publisher.publish(image->toImageMsg());

}

int main( int argc, char** argv)
{

  //Init ROS
  ros::init(argc, argv, "test_blur");
  ros::NodeHandle n;


  //Set up image publish and subscribe
  image_transport::ImageTransport it(n);

  image_transport::Subscriber sub = 
    it.subscribe("input",
		 1,
		 callback);

  publisher = it.advertise("output",1);

  string face_cascade_file;

  n.getParam("face_cascade_file", face_cascade_file);

  faces.load(face_cascade_file);

  //Transfer control to ROS
  ros::spin();

  return 0;

}

