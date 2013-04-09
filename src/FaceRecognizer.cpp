
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#define INPUT_TOPIC "camera/rgb/image_color"
#define OUTPUT_TOPIC "detector/blurred"

image_transport::Publisher publisher;

using namespace cv;

Ptr<FaceRecognizer> model;

void callback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  const sensor_msgs::Image img = *msg;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
  Mat cvImage = image->image;

  

}

int main( int argc, char** argv)
{

  //Init ROS
  ros::init(argc, argv, "test_blur");
  ros::NodeHandle n;


  //Set up image publish and subscribe
  image_transport::ImageTransport it(n);

  image_transport::Subscriber sub = 
    it.subscribe(INPUT_TOPIC,
		 1,
		 callback);

  vector<Mat> images;
  vector<int> labels;

  try {
    read_csv(CSV_FILE, images, labels);
  }

  model = createEigenFaceRecognizer();


  //Transfer control to ROS
  ros::spin();

  return 0;

}

