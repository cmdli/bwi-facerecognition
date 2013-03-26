
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#define INPUT_TOPIC "camera/rgb/image_color"
#define OUTPUT_TOPIC "detector/gradients"

image_transport::Publisher publisher;

using namespace cv;

void callback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  const sensor_msgs::Image img = *msg;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
  Mat cvImage = image->image;
  Mat cvGradientsX, cvGradientsY, cvGradients;


  //Create kernels
  Mat kernel_x = (Mat_<int>(1,3) << -1,0,-1);
  Mat kernel_y = (Mat_<int>(3,1) << -1,
		                    0,
		                    1);

  filter2D(cvImage, cvGradientsX, 1, kernel_x);
  filter2D(cvImage, cvGradientsY, 1, kernel_y);

  cvGradients.create(cvImage.rows,cvImage.cols,CV_8UC1);
  sqrt(cvGradientsX.mul(cvGradientsX) + cvGradientsY.mul(cvGradientsY), cvGradients);

  image->image = cvGradients;;
  
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

