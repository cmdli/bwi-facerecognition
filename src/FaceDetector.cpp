

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include <iostream>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>


image_transport::Publisher publisher;

using namespace std;
using namespace cv;

CascadeClassifier faces;

void callback(const sensor_msgs::ImageConstPtr &rgbMsg, const sensor_msgs::ImageConstPtr &depthMsg)
{
  //Load image into OpenCV
  const sensor_msgs::Image img = *rgbMsg;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(rgbMsg);
  cv::Mat cvImage = image->image;
 cv::Mat cvOutput;
 cv_bridge::CvImagePtr depthImage = cv_bridge::toCvCopy(depthMsg);
  cv::Mat cvDepth = depthImage->image;

  //cvImage.mul(cvDepth);

  cvtColor(cvImage, cvOutput, CV_RGB2GRAY);

  vector<Rect> faceRects;

  faces.detectMultiScale(cvOutput, faceRects);

  for(int i = 0; i < faceRects.size(); i++) {
    rectangle( cvDepth, faceRects[i], Scalar(0,255,0));
  }

  ROS_INFO("Publishing");
  
  //image->image = cvOutput;
  //image->encoding = "mono8";

  //Publish image
  publisher.publish(depthImage->toImageMsg());

}



int main( int argc, char** argv)
{
 ROS_INFO("Publishing");
  cout << "Starting node" << endl;
  cout.flush();

  //Init ROS
  ros::init(argc, argv, "face_detector");
  ros::NodeHandle n;


  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n,"rgb_input",1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(n,"depth_input",1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> KinectSyncPolicy;
  message_filters::Synchronizer<KinectSyncPolicy> sync(KinectSyncPolicy(10),rgb_sub,depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  /*image_transport::Subscriber sub = 
    it.subscribe("rgb_image",
		 1,
		 callback);*/




  //Set up image publish
  image_transport::ImageTransport it(n);
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

