

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
image_transport::Publisher facesPublisher;

using namespace std;
using namespace cv;

CascadeClassifier faces;

Ptr<FaceRecognizer> model;


#define DIFFERENCE_THRESHOLD 3500

Mat& scaleImage(Mat& image, int width, int height)
{
  cv::Mat scaled;
  cv::resize(image, scaled, Size(width,height), 0, 0);
  return scaled;
}

//Trains the face recognizer on labelled data
void train(string csv_file, string faces_path)
{
  ROS_INFO("Loading training data...");
  ROS_INFO(csv_file.c_str());

  vector<Mat> images;
  vector<int> labels;

  //Read the images and labels using the CSV file
  try {
    ifstream file(csv_file.c_str());
    if(!file) {
      string error_msg = "Could not load CSV file: " + csv_file;
      ROS_ERROR(error_msg.c_str());
	exit(1);
    }

    string line, relative_file_path, type;
    while(getline(file,line)) {
      //Exclude comments
      if(line[0] == '#')
	continue;

      //Get the image and the label
      stringstream stream(line);
      getline(stream, relative_file_path, ';');
      getline(stream, type);
      string file_path = faces_path + relative_file_path;

      if(!file_path.empty() && !type.empty()) {

	stringstream ss;
	ss << "Loading training file: " << file_path;
        ROS_INFO(ss.str().c_str());

	//Check if file actually exists
	ifstream file(file_path.c_str());
	if (!file.good()) {
		ss.clear();
		ss << "Unable to load file: " << file_path;
		ROS_ERROR(ss.str().c_str());
	}
	else {
	  file.close();

	  //Read in image and scale it
	  cv::Mat image = imread(file_path);
	  cv::Mat grayUnscaledFace;
	  cvtColor(image, grayUnscaledFace, CV_RGB2GRAY);

	  cv::Mat scaledFace = scaleImage(grayUnscaledFace, 105, 105);

	  //Store training image and label
	  images.push_back(scaledFace);
	  labels.push_back(atoi(type.c_str()));

	}
	
      }
      
    }
    
  }
  catch(Exception &e) {
    ROS_ERROR("Could not load face data");
  }

  //Train model on the loaded data
  ROS_INFO("Starting training");
  model = createEigenFaceRecognizer();
  model->train(images, labels);
  ROS_INFO("Done training");
}

int recognizeFace(Mat& image)
{
  double difference;
  int label;

  model->predict(image,label,difference);
  
  //If the model is unsure on the face, mark it as unrecognized
  if (difference > DIFFERENCE_THRESHOLD)
	label = -1;

  stringstream ss;
  ss << "Saw: " << label << "	Diff: " << difference;
  ROS_INFO(ss.str().c_str());

  return label;
}



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

  //Load faces from image
  vector<Rect> faceRects;
  faces.detectMultiScale(cvGray, faceRects);

  string encoding = image->encoding;

  //Draw boxes and label each face
  for(int i = 0; i < faceRects.size(); i++) {

    //If the face is too small, exclude it
    if (faceRects[i].width > 200)
	continue;

    //Extract the face fom the original image and scale it
    cv::Mat croppedFace = cvGray(faceRects[i]);
    cv::Mat scaledFace = scaleImage(croppedFace, 105, 105);

    //Publish the cropped faces
    image->image = croppedFace;
    image->encoding = "mono8";
    facesPublisher.publish(image->toImageMsg());

    //Recognize the person and draw a colored box around it
    int person = recognizeFace(scaledFace);
    String name;
    Scalar color;
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

  }

  image->image = cvImage;
  image->encoding = encoding;

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
  facesPublisher = it.advertise("cropped_faces",10);

  //Load in the path to the cascade file from the launch file
  string face_cascade_file;
  if(!n.getParam("/FaceDetector/face_cascade_file", face_cascade_file)) {
    ROS_ERROR("Could not find 'face_cascade_file' parameter");
    return 1;
  }

  //Load the cascade file for face detection
  faces.load(face_cascade_file);

  //Load in the CSV file with the training data for the face recognizer
  string csv_file;
  if(!n.getParam("/FaceDetector/csv_file", csv_file)) {
    ROS_ERROR("CSV file not set in launch file");
    return 1;
  }

  //Load the location of the training data
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

