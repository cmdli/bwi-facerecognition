

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include <stdlib.h>
#include <iostream>
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
image_transport::Publisher facesPublisher;

using namespace std;
using namespace cv;

CascadeClassifier faces;

Ptr<FaceRecognizer> model;

void train(string csv_file)
{
  ROS_INFO("Loading training data...");
  ROS_INFO(csv_file.c_str());

  vector<Mat> images;
  vector<int> labels;

  try {
    ifstream file(csv_file.c_str());
    if(!file) {
      string error_msg = "Could not load CSV file: " + csv_file;
      ROS_ERROR(error_msg.c_str());
	exit(1);
    }

    string line, file_path, type;
    while(getline(file,line)) {
      if(line[0] == '#')
	continue;

      stringstream stream(line);
      getline(stream, file_path, ';');
      getline(stream, type);
      if(!file_path.empty() && !type.empty()) {
	stringstream ss;
	ss << "Loading training file: " << file_path;
        ROS_INFO(ss.str().c_str());
        IplImage* img = cvLoadImage(file_path.c_str());
	cv::Mat image(img);
	cvReleaseImage(&img);
	cv::Mat grayUnscaledFace;
	cvtColor(image, grayUnscaledFace, CV_RGB2GRAY);
	ss.clear();
        cv::Mat scaledFace;
        cv::resize(grayUnscaledFace,scaledFace,Size(105,105),0,0);
	images.push_back(scaledFace);
	labels.push_back(atoi(type.c_str()));
      }
      
    }
    
  }
  catch(Exception &e) {
    ROS_ERROR("Could not load face data");
  }
  ROS_INFO("Starting training");
  model = createEigenFaceRecognizer();
  model->train(images, labels);
  ROS_INFO("Done training");
}

string recognizeFace(Mat image)
{
  int result = model->predict(image);
  stringstream ss;
  ss << "Saw: " << result;

  ROS_INFO(ss.str().c_str());

  

  return ss.str();
}

void callback(const sensor_msgs::ImageConstPtr &imgptr)
{


  //Load image into OpenCV
  const sensor_msgs::Image img = *imgptr;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(img);
  cv::Mat cvImage = image->image;
  cv::Mat cvGray;

  cvtColor(cvImage, cvGray, CV_RGB2GRAY);
  cv::Mat cvOutput(cvGray);

  vector<Rect> faceRects;

  faces.detectMultiScale(cvGray, faceRects);

  for(int i = 0; i < faceRects.size(); i++) {
    rectangle( cvOutput, faceRects[i], Scalar(255,0,0));
    cv::Mat croppedFace = cvGray(faceRects[0]);
    cv::Mat scaledFace;
    cv::resize(croppedFace,scaledFace,Size(105,105),0,0);
    image->image = croppedFace;
    image->encoding = "mono8";
    facesPublisher.publish(image->toImageMsg());
    recognizeFace(scaledFace);
  }

  image->image = cvOutput;
  image->encoding = "mono8";

  ROS_INFO("Publishing");
  
  image->image = cvOutput;
  image->encoding = "mono8";

  //Publish image
  publisher.publish(image->toImageMsg());

}



int main( int argc, char** argv)
{
  //Init ROS
  ros::init(argc, argv, "face_detector");
  ros::NodeHandle n;

  image_transport::ImageTransport it(n);

  image_transport::Subscriber sub = 
    it.subscribe("rgb_image",
		 1,
		 callback);


  publisher = it.advertise("output",10);
  facesPublisher = it.advertise("cropped_faces",10);

  string face_cascade_file;

  if(!n.getParam("/FaceDetector/face_cascade_file", face_cascade_file)) {
    ROS_ERROR("Could not find 'face_cascade_file' parameter");
    return 1;
  }

  faces.load(face_cascade_file);

  string csv_file;
  if(!n.getParam("/FaceDetector/csv_file", csv_file)) {
    ROS_ERROR("CSV file not set in launch file");
    return 1;
  }

  train(csv_file);

  //Transfer control to ROS
  ros::spin();

  return 0;

}

