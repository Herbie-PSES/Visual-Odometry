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
//#include <opencv2/highgui/highgui.cpp>
#include <opencv2/rgbd.hpp>
#include <iostream>


#include <ctime>

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
    void depth_imageCallback(const sensor_msgs::Image::ConstPtr &imagMsg);
private:
    ros::NodeHandle nh;
    image_transport::Subscriber image_sub;
    image_transport::Subscriber depth_image_sub;
    rgbd::RgbdOdometry rgbdOdometry;
     cv_bridge::CvImageConstPtr cv_ptr_image;
     cv_bridge::CvImageConstPtr cv_ptr_depth_image;
     Mat cameraMatrix;

     int _firstImage;
     bool _nextImage;
     //vector<Point2f> points_last, points_current;        //vectors to store the coordinates of the feature points
     Mat img_last, img_current;
     Mat depth_last, depth_current;
     //Ptr<FeatureDetector> detector;

     Mat Rt;
     vector<Mat> relativePoses; // maybe use vector instead

     bool isfound; // true if transformation is found

};

node_class::node_class():relativePoses(10)
{
    // Camera Matrix: K: [540.68603515625, 0.0, 479.75, 0.0, 540.68603515625, 269.75, 0.0, 0.0, 1.0]

    //float camera_parameters[] = {540.68603515625, 0.0, 479.75, 0.0, 540.68603515625, 269.75, 0.0, 0.0, 1.0}; // qhd parameters
    float camera_parameters[] = {366.3330993652344, 0.0, 253.05209350585938, 0.0, 366.3330993652344, 204.04150390625, 0.0, 0.0, 1.0}; // sd paramters
    cameraMatrix = Mat(3, 3, CV_32FC1, camera_parameters);
    cv_ptr_image = nullptr;
    cv_ptr_depth_image = nullptr;
    _firstImage =1;
    _nextImage = false;

    rgbdOdometry = rgbd::RgbdOdometry(cameraMatrix);


}

void node_class::loop_function()
{
    if(!_nextImage) return;
    _nextImage =false;
    if(cv_ptr_image == nullptr || cv_ptr_depth_image == nullptr){
        ROS_INFO("NULLPOINTER NO IMAGE");
    }
    else{

    Mat mask = Mat::zeros(img_current.size(), CV_8U);  // type of mask is CV_8U
   // Mat roi(mask, cv::Rect(10,10,900,500));
    if(_firstImage == 1){
        img_last = img_current;
        depth_last = depth_current;
        _firstImage =0;
    }
    clock_t begin = clock();

    cout << cv_ptr_depth_image->header <<endl << cv_ptr_image->header << endl;

        cv::imshow( "Display window", depth_last );
        cv::waitKey(3); // waits for key input
    isfound = rgbdOdometry.compute(img_last, depth_last, Mat(), img_current, depth_current, Mat(), Rt); // Standard init is last Rt

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    if(isfound){
   ROS_INFO("Transformation found: \n Elapsed time: %f s ",elapsed_secs);
    } else {
        ROS_INFO("Transformation not found");
    }

    img_last = img_current;
    depth_last = depth_current;
    cout << Rt << endl;
    }
}

void node_class::imageCallback(const sensor_msgs::Image::ConstPtr& imagMsg)
{

    cv_ptr_image = cv_bridge::toCvCopy(imagMsg, "mono8"); // 8-Bit Greyscale Image
    img_current = cv_ptr_image->image;// korrigieren wegen Pointer und effizienz
    _nextImage = true;
}

void node_class::depth_imageCallback(const sensor_msgs::Image::ConstPtr& imagMsg)
{

    cv_ptr_depth_image = cv_bridge::toCvCopy(imagMsg, imagMsg->encoding ); //"32FC1"
    cv_ptr_depth_image->image.convertTo(depth_current,CV_32FC1,0.001);
    //depth_current = cv_ptr_depth_image->image / 1000.0; //mm to m
    double min, max;
    cv::minMaxLoc(depth_current, &min, &max);

   // for(int y=0;y<imageHeight;y++){
     //   for(int x=0;x<imageWidth;x++){
       //     if(depth_current.at(x,y)==min || depth_current.at(x,y)==max)
         //       depth_current.
    //}}

    //for(float p : depth_current){
       //     if(depth_current.at(x,y)==min || depth_current.at(x,y)==max)
     //           p = 0.0/0.0;
   // }

    /*
    for (int row = 0; row < depth_current.rows; ++row)
    {
        const uchar *ptr = depth_current.ptr(row);
        for (int col = 0; col < depth_current.cols; col++)
        {
            const uchar * uc_pixel = ptr;

            if(uc_pixel[0] == min || uc_pixel[0] ==max)
                uc_pixel[0] == 0.0/0.0;
            ptr += 1;
        }
    }*/

    Size contSize = depth_current.size();
    if (depth_current.isContinuous())
    {
        contSize.width *= contSize.height;
        contSize.height = 1;
    }
    for (int i = 0; i < contSize.height; ++i)
    {
        float* ptr = depth_current.ptr<float>(i);
        for (int j = 0; j < contSize.width; ++j)
        {   if(ptr[j] == min || ptr[j] ==max)
            ptr[j] = 0.0/0.0;
        }
    }

    cv::minMaxLoc(depth_current, &min, &max);

    cout << "Depth"<< depth_current.at<float>(480,270) << endl;
    cout << "min:   " << min << "   max:   " <<max <<endl;
   // imwrite("Depthimage.png",cv_ptr_depth_image->image);
   // cout << depth_current << endl;


}

void node_class::run_once()
{
    image_transport::ImageTransport i_trans(nh);
    image_sub = i_trans.subscribe("/kinect2/sd/image_color_rect", 1, &node_class::imageCallback, this);
    depth_image_sub = i_trans.subscribe("/kinect2/sd/image_depth_rect",1,&node_class::depth_imageCallback,this);
    // detector = ORB::create(); // add parameters if needed
     cv::namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_odometry_node");
  node_class this_node;
  this_node.run_once();

ros::Rate loop_rate(5);
  while (ros::ok()) {
      this_node.loop_function();
      ros::spinOnce();
      loop_rate.sleep();

  }

  return 0;
}
