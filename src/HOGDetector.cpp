
#define INPUT_TOPIC "camera/rgb/image_color"
#define OUTPUT_TOPIC "detector/blurred"

image_transport::Publisher publisher;

using namespace cv;


#define GRADIENT_KERNEL_X {-1,0,1}

void callback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  const sensor_msgs::Image img = *msg;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
  cv::Mat cvImage = image->image;

  
  Mat gradientX, gradientY, gradientMag, gradientAng; 

  Mat gradientKernelX(1,3,CV_UC1,GRADIENT_KERNEL_X);
  Mat gradientKernelY = gradientKernelX.t(); //Transpose

  filter2D(cvImage,gradientX,gradientKernelX);
  filter2D(cvImage,gradientY,gradientKernelY);

  sqrt(gradientX.mul(gradientX) + gradientY.mul(gradientY),gradientMag);
  gradientAng = atan2(gradientX,gradientY);

  //Convert gradient magnitudes to image
  Mat output;
  for(int i = 0; i < gradientMag.rows; i++) {
    for(int j = 0; j < gradientMag.cols; j++) {
      output.at<uchar>(i,j) = (uchar)gradientMag.at<double>(i,j);
    }
  }

  output = cvtColor(
  
  image->image = output;
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
    it.subscribe(INPUT_TOPIC,
		 1,
		 callback);

  publisher = it.advertise(OUTPUT_TOPIC,1);

  //Transfer control to ROS
  ros::spin();

  return 0;

}

