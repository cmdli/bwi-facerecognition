
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"


#define INPUT_TOPIC "camera/rgb/image_color"
#define OUTPUT_TOPIC "detector/SIFT"

image_transport::Publisher publisher;

using namespace std;
using namespace cv;

void callback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  const sensor_msgs::Image img = *msg;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
  cv::Mat cvImage = image->image;
  cv::Mat cvOutput; 

  vector<KeyPoint> points;
  Ptr<FeatureDetector> detector = FeatureDetector::create("SimpleBlob");
  detector->detect(cvImage, points);

  drawKeypoints(cvImage, points, cvImage);
  
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
    it.subscribe(INPUT_TOPIC,
		 1,
		 callback);

  publisher = it.advertise(OUTPUT_TOPIC,1);

  //Transfer control to ROS
  ros::spin();

  return 0;

}

