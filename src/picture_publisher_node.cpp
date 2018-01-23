#include <ros/ros.h>
#include <std_msgs/String.h>
#include  <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/core/ocl.hpp>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "picture_publisher_node");
  ros::NodeHandle nh;

  cv::Mat im_to_publish1;
  cv::Mat im_to_publish2;

  image_transport::ImageTransport it(nh);
  //ros::Publisher picture_pub = nh.advertise<sensor_msgs::Image>("/kinect2/qhd/image_mono_rect", 1);
  image_transport::Publisher pub = it.advertise("/kinect2/qhd/image_mono_rect", 1);

  //cv::namedWindow( "Display window", WINDOW_AUTOSIZE );
  ros::Rate loop_rate(5);

  int numFrame = 0;
  while (ros::ok())
  {
        char filename[100];

      sprintf(filename, "/home/maurice/Bilder/test_pics/im%d.png", numFrame);
      ROS_INFO(filename);
      im_to_publish1 = cv::imread(filename,CV_LOAD_IMAGE_GRAYSCALE);  // specify input images

      if( im_to_publish1.empty() ) { printf("Error loading first image \n"); return -1; }

      //cv::imshow( "Display window", im_to_publish1 );
      //cv::waitKey(3); // waits for key input

    //sensor_msgs::Image picture;
    sensor_msgs::ImagePtr ppicture = cv_bridge::CvImage(std_msgs::Header(), "mono8", im_to_publish1).toImageMsg();

    numFrame++;
    if(numFrame==12) numFrame=0;


    pub.publish(ppicture);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
