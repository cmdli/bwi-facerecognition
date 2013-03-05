
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include <iostream>

#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/common/GreyscaleLuminanceSource.h>
#include <zxing/common/Counted.h>
#include <zxing/common/GlobalHistogramBinarizer.h>

image_transport::Publisher image_pub;

zxing::qrcode::QRCodeReader reader;

void blurCB(const sensor_msgs::ImageConstPtr &msg)
{
  const sensor_msgs::Image img = *msg;

  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
  cv::Mat cvImage = image->image;
  cv::Mat cvOutput;

  cv::cvtColor(cvImage, cvOutput, CV_RGB2GRAY);

  zxing::GreyscaleLuminanceSource lum_source(cvOutput.data,
					     cvOutput.cols,
					     cvOutput.rows,
					     0, 0, 
					     cvOutput.cols,
					     cvOutput.rows);
  zxing::Ref<zxing::GreyscaleLuminanceSource> source_ref(&lum_source);
  zxing::GlobalHistogramBinarizer binarizer(source_ref);
  zxing::Ref<zxing::Binarizer> binarizer_ref((zxing::Binarizer*) &binarizer);
  zxing::BinaryBitmap bitmap(binarizer_ref);
  zxing::Ref<zxing::BinaryBitmap> bitmap_ref(bitmap);

  //reader.decode(zxing::Ref<zxing::BinaryBitmap>(&bitmap),zxing::DecodeHints::DEFAULT_HINT);

  zxing::Ref<zxing::Result> result = reader.decode(zxing::Ref<zxing::BinaryBitmap>(&bitmap),zxing::DecodeHints::DEFAULT_HINT);
					   

  std::cout << "Recieved image..." << std::endl;
  std::cout << "Text: " << (*result).getText() << std::endl;

  /*image->image = cvOutput;
  image->encoding = "mono8";

  std::cout << "Recieved image..." << std::endl;

  image_pub.publish(image->toImageMsg());*/
  
}

int main( int argc, char** argv)
{

  ros::init(argc, argv, "test_blur");
  ros::NodeHandle n;

  image_transport::ImageTransport it(n);

  image_pub = it.advertise("blurred_image",1);

  image_transport::Subscriber sub = 
    it.subscribe("kinect/rgb/image_color",
		 1,
		 blurCB);

  ros::Rate loop_rate(10);

  ros::spin();

  return 0;

}
