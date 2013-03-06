
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include <iostream>

#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/common/GreyscaleLuminanceSource.h>
#include <zxing/common/Counted.h>
#include <zxing/common/GlobalHistogramBinarizer.h>

using namespace std;
using namespace zxing;
using namespace zxing::qrcode;

void blurCB(const sensor_msgs::ImageConstPtr &msg)
{
 
 
  //A buffer containing an image. In your code, this would be an image from your camera. In this 
     // example, it's just an array containing the code for "Hello!". 
  /*try
	{
	  /*uint8_t buffer[] = 
    { 
      255, 255, 255, 255, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 
      255,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 255,  0 ,  0 ,  0 ,  0 ,  0 , 255,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 255, 
      255,  0 , 0, 0, 0, 0, 0,  0 , 0, 255,  0 , 255,  0 , 255, 255,  0 , 255, 255, 255, 255, 255,  0 , 255, 
      255,  0 , 0,  0 ,  0 ,  0 , 0,  0 , 255, 255,  0 ,  0 , 255, 255, 255,  0 , 255,  0 ,  0 ,  0 , 255,  0 , 255, 
      255,  0 , 0,  0 ,  0 ,  0 , 0,  0 , 255, 255,  0 ,  0 ,  0 , 255, 255,  0 , 255,  0 ,  0 ,  0 , 255,  0 , 255, 
      255,  0 , 0,  0 ,  0 ,  0 , 0,  0 , 255, 255, 255,  0 , 255,  0 , 255,  0 , 255,  0 ,  0 ,  0 , 255,  0 , 255, 
      255,  0 , 0, 0, 0, 0, 0,  0 , 0, 255, 255, 255,  0 ,  0 , 255,  0 , 255, 255, 255, 255, 255,  0 , 255, 
      255,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 255,  0 , 255,  0 , 255,  0 , 255,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 255, 
      255, 255, 255, 255, 255, 0, 0, 0, 255,  0 ,  0 ,  0 ,  0 ,  0 , 255, 255, 255, 255, 255, 255, 255, 255, 255, 
      255,  0 ,  0 , 255,  0 ,  0 , 255,  0 , 255, 255,  0 ,  0 , 255,  0 , 255,  0 , 255, 255, 255, 255, 255,  0 , 255, 
      255,  0 ,  0 ,  0 ,  0 ,  0 , 255, 255,  0 , 255, 255,  0 , 255,  0 , 255,  0 , 255,  0 ,  0 ,  0 , 255, 255, 255, 
      255, 255, 255, 255, 255, 255, 255,  0 , 255,  0 ,  0 ,  0 , 255, 255,  0 ,  0 , 255,  0 , 255,  0 ,  0 ,  0 , 255, 
      255, 255, 255,  0 ,  0 , 255, 255, 255,  0 ,  0 ,  0 , 255,  0 , 255, 255, 255, 255, 255, 255,  0 , 255, 255, 255, 
      255,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 255, 255,  0 , 255, 255, 255, 255,  0 , 255, 255,  0 , 255,  0 , 255, 255, 
      255, 255, 255, 255, 255, 255, 255, 255, 255,  0 ,  0 , 255,  0 ,  0 , 255, 255, 255, 255,  0 , 255,  0 ,  0 , 255, 
      255,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 255, 255, 255,  0 , 255,  0 ,  0 ,  0 ,  0 ,  0 , 255,  0 , 255, 255, 255, 
      255,  0 , 255, 255, 255, 255, 255,  0 , 255, 255,  0 ,  0 ,  0 ,  0 ,  0 , 255,  0 ,  0 ,  0 ,  0 , 255, 255, 255, 
      255,  0 , 255,  0 ,  0 ,  0 , 255,  0 , 255,  0 , 255, 255,  0 ,  0 , 255,  0 ,  0 , 255,  0 , 255,  0 ,  0 , 255, 
      255,  0 , 255,  0 ,  0 ,  0 , 255,  0 , 255,  0 , 255,  0 ,  0 ,  0 , 255, 255, 255, 255, 255, 255, 255, 255, 255, 
      255,  0 , 255,  0 ,  0 ,  0 , 255,  0 , 255, 255,  0 ,  0 , 255, 255, 255,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 255, 
      255,  0 , 255, 255, 255, 255, 255,  0 , 255,  0 , 255,  0 , 255, 255,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 255, 
      255,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 255,  0 , 255,  0 ,  0 , 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 
      255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255
    }; 
  int width = 23; 
  int height = 23;*/
  /*const sensor_msgs::Image img = *msg;

  cv_bridge::CvImagePtr cvimageptr = cv_bridge::toCvCopy(msg);
  cv::Mat cvImage = cvimageptr->image;
  cv::Mat cvOutput;

  cv::cvtColor(cvImage, cvOutput, CV_RGB2GRAY);

  cv::Size size = cvOutput.size();
  unsigned char *buffer = cvOutput.ptr();
  int width = size.width;
  int height = size.height;
 
  // Convert the buffer to something that the library understands. 
  Ref<LuminanceSource> source (new GreyscaleLuminanceSource(buffer,width,height,0,0,width,height)); 
 
  // Turn it into a binary image. 
  Ref<Binarizer> binarizer (new GlobalHistogramBinarizer(source)); 
  Ref<BinaryBitmap> image(new BinaryBitmap(binarizer));
 
  // Tell the decoder to try as hard as possible. 
  DecodeHints hints(DecodeHints::DEFAULT_HINT); 
  hints.setTryHarder(true); 
 
  // Perform the decoding. 
  QRCodeReader reader;
  Ref<Result> result(reader.decode(image, hints));
 
  // Output the result. 
  cout << result->getText()->getText() << endl;
 }
catch (zxing::Exception& e) 
	{
    cerr << "Error: " << e.what() << endl;
  }*/

    const sensor_msgs::Image img = *msg;

  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
  cv::Mat cvImage = image->image;
  cv::Mat cvOutput;

  cv::cvtColor(cvImage, cvOutput, CV_RGB2GRAY);

  cv::Size size = cvOutput.size();

  std::cout << "Crahs?" << std::endl;

  try {
    Ref<LuminanceSource> source(new GreyscaleLuminanceSource(cvOutput.ptr(),size.width,size.height,0,0,size.width,size.height));

    Ref<Binarizer> binarizer(new GlobalHistogramBinarizer(source));
    Ref<BinaryBitmap> bitmap(new BinaryBitmap(binarizer));
    DecodeHints hints(DecodeHints::BARCODEFORMAT_QR_CODE_HINT); 
    hints.setTryHarder(true); 
    QRCodeReader reader;
    Ref<Result> result(reader.decode(bitmap,hints));
    cout << "Detected QR Code!" << endl;
    cout << "Result: " << result->getText()->getText() << endl;
  }
  catch(Exception &e) {
    cout << "No QR Code" << endl;
  }

  cout << "-----------------------------------------------------" << endl;
}

int main( int argc, char** argv)
{

  ros::init(argc, argv, "test_blur");
  ros::NodeHandle n;

  image_transport::ImageTransport it(n);

  image_transport::Subscriber sub = 
    it.subscribe("camera/rgb/image_color",
		 1,
		 blurCB);

  ros::Rate loop_rate(10);

  ros::spin();

  return 0;

}
