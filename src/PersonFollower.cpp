
#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#define INPUT_TOPIC "kinect/rgb/image_color"

#define MIN_WIDTH 100

#define HALF_X 320.0
#define Z_THRESHOLD (HALF_X*0.1)
#define Z_MAX_SPEED 1.0

#define BALL_WIDTH 180.0
#define X_THRESHOLD (BALL_WIDTH*0.1)
#define X_MAX_SPEED 4.0

using namespace std;
using namespace cv;
using namespace ros;

Publisher velocity_pub;
HOGDescriptor detector;

void callback(const sensor_msgs::ImageConstPtr &msg)
{
  //Load image into OpenCV
  const sensor_msgs::Image img = *msg;
  cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);
  Mat cvImage = image->image;

  vector<Rect> persons;
  detector.detectMultiScale(cvImage, persons);

  geometry_msgs::Twist output;
  output.linear.x = 0;
  output.angular.z = 0;

  int highestI = -1;
  int largestWidth = 0;

  for (int i = 0; i < persons.size(); i++) {
    if(persons[i].width > largestWidth) {
      highestI = i;
      largestWidth = persons[i].width;
    }
  }

  cout << "---------------------------------------------" << endl << endl;

  if(highestI != -1) {
    int i = highestI;
    
    cout << "Person Detected!" << endl;
    cout << "\t-X: " << persons[i].x << endl;
    cout << "\t-Y: " << persons[i].y << endl;
    cout << "\t-W: " << persons[i].width << endl;
    cout << "\t-H: " << persons[i].height << endl;

    if(largestWidth >= MIN_WIDTH) {

      //Z
      /*      double diff = persons[i].x - HALF_X;
      double motorZ = -(diff)/HALF_X * Z_MAX_SPEED;

      motorZ *= motorZ < 0 ? -motorZ : motorZ;

      if(abs(diff) < Z_THRESHOLD)
	motorZ = 0;

      output.angular.z = motorZ;*/

      //X
      double Xdiff = persons[i].width - BALL_WIDTH;
      double motorX = -(Xdiff)/BALL_WIDTH * X_MAX_SPEED;

      motorX *= motorX < 0 ? -motorX : motorX;

      if(abs(Xdiff) < X_THRESHOLD)
	motorX = 0;

      output.linear.x = motorX; 

      cout << endl << endl;
      cout << "Following person with..." << endl;
      cout << "\t-X: " << output.linear.x << endl;
      cout << "\t-Z: " << output.angular.z << endl;

    }

  }

  velocity_pub.publish(output); 

}

int main( int argc, char* argv[])
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

  velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  detector.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

  //Transfer control to ROS
  ros::Rate loop_rate(10);
  ros::spin();

  return 0;

}

