

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
    rectangle( cvOutput, faceRects[i], Scalar(0,255,0));
  }
  
  image->image = cvOutput;
  image->encoding = "mono8";

  //Publish image
  publisher.publish(image->toImageMsg());

}

int main( int argc, char** argv)
{
  cout << "Starting node" << endl;
  cout.flush();

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

  if(!n.getParam("/FaceDetector/face_cascade_file", face_cascade_file)) {
    ROS_ERROR("Could not find 'face_cascade_file' parameter");
    return 1;
  }

  faces.load(face_cascade_file);

  //Transfer control to ROS
  ros::spin();

  return 0;

}

