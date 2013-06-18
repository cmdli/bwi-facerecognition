

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>

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

#include <stdint.h>


image_transport::Publisher publisher;
image_transport::Publisher facesPublisher;

using namespace std;
using namespace cv;

CascadeClassifier faces;

Ptr<FaceRecognizer> model;

void train(string csv_file, string faces_path)
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

    string line, relative_file_path, type;
    while(getline(file,line)) {
      if(line[0] == '#')
	continue;

      stringstream stream(line);
      getline(stream, relative_file_path, ';');
      getline(stream, type);
      string file_path = faces_path + relative_file_path;

      if(!file_path.empty() && !type.empty()) {

	stringstream ss;
	ss << "Loading training file: " << file_path;
        ROS_INFO(ss.str().c_str());

	ifstream file(file_path.c_str());
<<<<<<< HEAD
	if (!file.good()) {
	  ss.clear();
	  ss << "Unable to load file: " << file_path;
	  ROS_ERROR(ss.str().c_str());
=======
	if (file.good())
	{
		file.close();
		cv::Mat image = imread(file_path);
		cv::Mat grayUnscaledFace;
		cvtColor(image, grayUnscaledFace, CV_RGB2GRAY);

        	cv::Mat scaledFace;
        	cv::resize(grayUnscaledFace,scaledFace,Size(105,105),0,0);

		images.push_back(scaledFace);
		labels.push_back(atoi(type.c_str()));
	}
	else {
		ss.clear();
		ss << "Unable to load file: " << file_path;
		ROS_ERROR(ss.str().c_str());
>>>>>>> e4a75ae0fbcc0ef4e00511bb5f9890fa7ed8e9ad
	}
	
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

int recognizeFace(Mat image)
{
  double difference;
  int label;

  model->predict(image,label,difference);
<<<<<<< HEAD
  
  //If the model is unsure on the face, mark it as unrecognized
  if (difference > DIFFERENCE_THRESHOLD)
    label = -1;
=======
  if (difference > 4000)
	label = -1;
>>>>>>> e4a75ae0fbcc0ef4e00511bb5f9890fa7ed8e9ad

  stringstream ss;
  ss << "Saw: " << label << "	Diff: " << difference;
  ROS_INFO(ss.str().c_str());

  return label;
}

bool isActuallyFace(cv::Mat depthMat, Rect faceRect) {
	float intensity = depthMat.at<float>(Point(0, 0));
	//uchar blue = intensity.val[0];
	//uchar red = intensity.val[2];
	//uchar green = intensity.val[1];
	cout << "Top left color is: " << intensity << endl;
	return true;
}

void callback(const sensor_msgs::ImageConstPtr &rgb_image_input, const sensor_msgs::ImageConstPtr &depth_image_input)
{
  //Load image into OpenCV
  //const  rgb_img_msg = *rgb_image_input;
  cv_bridge::CvImagePtr rgb_image = cv_bridge::toCvCopy((sensor_msgs::Image)*rgb_image_input);
  cv::Mat cvImage = rgb_image->image;

  //const  depth_img_msg = *depth_image_input;
  cv_bridge::CvImagePtr depth_image = cv_bridge::toCvCopy((sensor_msgs::Image)*depth_image_input);
  cv::Mat cvDepth = depth_image->image;

  cv::Mat cvGray;

  cvtColor(cvImage, cvGray, CV_RGB2GRAY);
  cv::Mat cvOutput(cvGray);

  vector<Rect> faceRects;

  faces.detectMultiScale(cvGray, faceRects);

  string oldEncoding = rgb_image->encoding;

  isActuallyFace(cvDepth, Rect(0,0,0,0));

<<<<<<< HEAD
    //If the face is too small, exclude it
    if (faceRects[i].width > 200)
      continue;
=======
  for(int i = 0; i < faceRects.size(); i++) {
    
>>>>>>> e4a75ae0fbcc0ef4e00511bb5f9890fa7ed8e9ad

    cv::Mat croppedFace = cvGray(faceRects[i]);
    cv::Mat scaledFace;
    cv::resize(croppedFace,scaledFace,Size(105,105),0,0);

    rgb_image->image = croppedFace;
    rgb_image->encoding = "mono8";

    facesPublisher.publish(rgb_image->toImageMsg());
    int person = recognizeFace(scaledFace);
    String name;
    Scalar color;
<<<<<<< HEAD
    bool personUnkown = false;
    switch (person) {
    case 0: 
      color = Scalar(0,255, 0); name = "Chris"; break;
    case 1: 
      color = Scalar(255,0,0); name = "Mukund"; break;
    case 2: 
      color = Scalar(0,128,255); name = "Ethan"; break;
    default: 
      color = Scalar(0,0,255); name = "Unknown"; break;
    }

    rectangle( cvImage, faceRects[i], color);	
    putText( cvImage, name.c_str(), Point(faceRects[i].x,faceRects[i].y+faceRects[i].height+20),  cv::FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,0), 2);
=======
	switch (person) {
		case 1: color = Scalar(0,255,0); name = "Ethan"; break;
		case 2: color = Scalar(0,0,255); name = "Mukund"; break;
		default: color = Scalar(255,0,0); name = "Unknown"; break;
	}
	rectangle( cvImage, faceRects[i], color);	
	putText( cvImage, name.c_str(), Point(faceRects[i].x,faceRects[i].y+faceRects[i].height+20),  cv::FONT_HERSHEY_PLAIN, 2.2, color, 3);
>>>>>>> e4a75ae0fbcc0ef4e00511bb5f9890fa7ed8e9ad

  }

  depth_image->image = cvDepth;
  //rgb_image->encoding = "bgr8";
  //depth_image->encoding = "mono8";

  //Publish image
  publisher.publish(depth_image->toImageMsg());

}



int main( int argc, char** argv)
{
  //Init ROS
  ros::init(argc, argv, "face_detector");
  ros::NodeHandle n;

  typedef image_transport::SubscriberFilter ImageSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  
  image_transport::ImageTransport it(n);
  ImageSubscriber rgb_input(it, "rgb_input", 1);
  ImageSubscriber depth_input(it, "depth_input", 1);
	
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_input, depth_input);
  sync.registerCallback(boost::bind(&callback, _1, _2));


  /*image_transport::Subscriber sub = 
  it.subscribe("rgb_input", 1, callback);*/



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

  string faces_path;
  if(!n.getParam("/FaceDetector/training_data_path", faces_path)) {
    ROS_ERROR("Training data file not set in launch file");
    return 1;
  }

  train(csv_file,faces_path);



  ROS_INFO("Transfer control to ROS");
  ros::spin();

  return 0;

}

