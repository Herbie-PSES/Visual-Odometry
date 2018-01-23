#include <ros/ros.h>
#include <std_msgs/String.h>
#include  <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <image_transport/image_transport.h>
//#include <opencv/highgui.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include <ctime>
#include <iostream>

#include "vo_features.h"

//#include <opencv2/highgui.hpp> which one???????????? looks like above includes that one implicit


using namespace cv;
using namespace std;

class node_class
{
public:
    node_class();
    void loop_function();
    void run_once();
    void imageCallback(const sensor_msgs::Image::ConstPtr &imagMsg);//imageCallback
private:
    ros::NodeHandle nh;
    image_transport::Subscriber image_sub;
     cv_bridge::CvImageConstPtr cv_ptr;

     int numFrame;
};

node_class::node_class()
{

    cv_ptr = nullptr;
    numFrame =0;

}

void node_class::loop_function()
{

}

void node_class::imageCallback(const sensor_msgs::Image::ConstPtr& imagMsg)
{

    cv_ptr = cv_bridge::toCvCopy(imagMsg, "mono8"); // 8-Bit Greyscale Image


    char filename[100];

  sprintf(filename, "/home/maurice/Bilder/aufnahmen_bag/%04d.jpg", numFrame);
  numFrame++;

    imwrite(filename,cv_ptr->image);




}

void node_class::run_once()
{
    image_transport::ImageTransport i_trans(nh);
    image_sub = i_trans.subscribe("/kinect2/qhd/image_color", 1, &node_class::imageCallback, this);

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_odometry_node");
  node_class this_node;
  this_node.run_once();


  while (ros::ok()) {
      ros::spinOnce();
  }

  return 0;
}
