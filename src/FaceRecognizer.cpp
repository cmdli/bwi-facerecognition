
#include <ros/ros.h>

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"

image_transport::Publisher publisher;

using namespace cv;
using namespace std;

Ptr<FaceRecognizer> model;

void train(string csv_file)
{
  ROS_INFO("Loading training data...");
  vector<Mat> images;
  vector<int> labels;

  try {
    ifstream file(csv_file.c_str());
    if(!file) {
      string error_msg = "Could not load CSV file: " + csv_file;
      ROS_ERROR(error_msg.c_str());
      return 1;
    }

    string line, file_path, type;
    while(getline(file,line)) {
      if(line[0] == '#')
	continue;

      stringstream stream(line);
      getline(stream, file_path, ';');
      getline(stream, type);
      if(!file_path.empty() && !type.empty()) {
	images.push_back(imread(file_path, 0));
	labels.push_back(atoi(type.c_str()));
      }
      
    }
    
  }
  catch(Exception &e) {
    ROS_ERROR("Could not load face data");
  }

   ROS_INFO("Training...");
  model = createEigenFaceRecognizer();
  model->train(images, labels);
}

void recognizeFace(const sensor_msgs::ImageConstPtr &msg)
{
  const sensor_msgs::Image img = *msg;
  cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(msg);
  Mat image = imgPtr->image;
  
  int result = model->predict(image);

  string output = string("Saw: ") + string(result);

  ROS_INFO(output.c_str());
}

int main( int argc, char** argv)
{

  //Init ROS
  ros::init(argc, argv, "FaceRecognizer");
  ros::NodeHandle n;

  //Set up image publish and subscribe
  image_transport::ImageTransport it(n);

  image_transport::Subscriber sub = 
    it.subscribe(INPUT_TOPIC,
		 1,
		 recognizeFace);

  string csv_file;
  if(!n.getParam("/FaceRecognizer/csv_file", csv_file)) {
    ROS_ERROR("CSV file not set in launch file");
    return 1;
  }

  train(n, csv_file);

  //God this code hurts
  //All this to print out an integer
  ostringstream ss;
  ss << "Predicted type: " << result << "\tActual type: " << testType;
  string output = ss.str();
  ROS_INFO(output.c_str());

  return 0;

}

