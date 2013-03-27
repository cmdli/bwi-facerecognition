
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include <iostream>

#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/common/GreyscaleLuminanceSource.h>
#include <zxing/common/Counted.h>
#include <zxing/common/GlobalHistogramBinarizer.h>
#include <zxing/qrcode/detector/FinderPatternFinder.h>
#include <zxing/qrcode/detector/FinderPatternInfo.h>
#include <zxing/qrcode/detector/FinderPattern.h>

#define INPUT_TOPIC "camera/rgb/image_color"
#define OUTPUT_TOPIC "detector/qrfinderpattern"

image_transport::Publisher publisher;

using namespace cv;
using namespace std;
using namespace zxing;
using namespace zxing::qrcode;

void callback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  const sensor_msgs::Image img = *msg;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
  cv::Mat cvImage = image->image;
  cv::Mat cvOutput; 

  cvtColor(cvImage, cvImage, CV_RGB2GRAY);

  Size size = cvImage.size();

  try {
    Ref<LuminanceSource> source(new GreyscaleLuminanceSource(cvOutput.ptr(),size.width,size.height,0,0,size.width,size.height));

    Ref<Binarizer> binarizer(new GlobalHistogramBinarizer(source));
    Ref<BinaryBitmap> bitmap(new BinaryBitmap(binarizer));
    DecodeHints hints(DecodeHints::BARCODEFORMAT_QR_CODE_HINT); 
    hints.setTryHarder(true); 

    Ref<BitMatrix> image(bitmap->getBlackMatrix());
    FinderPatternFinder finder(image, hints.getResultPointCallback());
    Ref<FinderPatternInfo> info(finder.find(hints));

    Ref<FinderPattern> topLeft(info->getTopLeft());
    circle(cvImage, Point(topLeft->getX(), topLeft->getY()), 3, Scalar(255,0,0));

  }
  catch(...) {
    int x = 1;
  }
  
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

