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

     int _firstImage;
     vector<Point2f> points_last, points_current;        //vectors to store the coordinates of the feature points
     Mat img_last, img_current;
     Ptr<FeatureDetector> detector;
     Mat cameraMatrix;
     vector<Mat> poses;

};

node_class::node_class()
{

    cv_ptr = nullptr;
    _firstImage =1;
    float camera_parameters[10] = {540.68603515625, 0.0, 479.75, 0.0, 540.68603515625, 269.75, 0.0, 0.0, 1.0};
    cameraMatrix = Mat(3, 3, CV_32F, camera_parameters);
    poses.push_back(Mat::eye(4,4,CV_32F));

}

void node_class::loop_function()
{

    if(cv_ptr == nullptr){
        ROS_INFO("NULLPOINTER NO IMAGE");
    }
}

void node_class::imageCallback(const sensor_msgs::Image::ConstPtr& imagMsg)
{
     clock_t begin = clock();



    cv_ptr = cv_bridge::toCvCopy(imagMsg, "mono8"); // 8-Bit Greyscale Image
    img_current = cv_ptr->image;// korrigieren wegen Pointer und effizienz

    // feature detection, tracking

    //featureDetection(cv_ptr->image, points1);        //detect features in img_1
    vector<uchar> status;
    Scalar color = Scalar(0,0,255);
    cv::Mat outimage;
    vector<KeyPoint> keypoints_current;

    //this is the FAST detector which is fast :D
    int fast_threshold = 15; //default 20
    bool nonmaxSuppression = true;
    FAST(img_current, keypoints_current, fast_threshold, nonmaxSuppression);

    //transform Keypoints to points for usage of vo_features.h tracking implementation
    KeyPoint::convert(keypoints_current, points_current, vector<int>());

    if(_firstImage == 1){
    points_last = points_current;
    img_last = img_current;
    _firstImage=0;
    return;
    }

    FAST(img_last, keypoints_current, fast_threshold, nonmaxSuppression);

    //transform Keypoints to points for usage of vo_features.h tracking implementation
    KeyPoint::convert(keypoints_current, points_last, vector<int>());

    // at the moment features get less with each picture, because the old keypoints of first picture are matched overall

    //featureTracking(img_last, img_current, points_last, points_current, status);

    img_last = img_current; // could become problematic

    /* HERE is alternative using ORB...
     *
    Mat mask = Mat::zeros(img_current.size(), CV_8U);  // type of mask is CV_8U
    Mat roi(mask, cv::Rect(10,10,100,100));
    roi = Scalar(255, 255, 255);
    Mat descriptor1;
    detector->detectAndCompute(img_current,mask,keypoints_1,descriptor1);
    */

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
   ROS_INFO("Elapsed time: %f s",elapsed_secs);


    /* Draw features if Point is used
    vector<Point2f>::iterator it;

    for(it = points_current.begin(); it!=points_current.end();it++)
    {
    cv::circle(img_current,*it,4,color);
    }
    */

    //Draw optical Flow
    cvtColor(img_current,outimage,CV_GRAY2BGR);

    for(int i =0; i<points_current.size();i++)
    {
    cv::line(outimage,points_current.at(i),points_last.at(i),color,2);
    }

    //drawKeypoints(img_current, keypoints_current, outimage, color);





    //recovering the pose and the essential matrix
    Mat E, R, t, mask;
    E = findEssentialMat(points_current, points_last, cameraMatrix, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, points_current, points_last,cameraMatrix, R, t, mask);
    //recover Angles from R
    Mat r;
    Rodrigues(R,r);
    Mat R_t;
    hconcat(R,t,R_t);
    vconcat(R_t,Mat(1,4,CV_32F,{0,0,0,1}),R_t);
    R_t = R_t*poses.back();
    poses.push_back(R_t);
    /*
    // baue t_f
    Mat traj = Mat::zeros(600, 600, CV_8UC3);
    int x = int(t_f.at<double>(0)) + 300;
    int y = int(t_f.at<double>(2)) + 100;
    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

    imshow( "Trajectory", traj );

    waitKey(1);
    */

    cv::imshow( "Display window", outimage );
    cv::waitKey(3); // waits for key input

    cout << r << endl;
    cout << t << endl;


}

void node_class::run_once()
{
    image_transport::ImageTransport i_trans(nh);
    image_sub = i_trans.subscribe("/kinect2/qhd/image_color", 1, &node_class::imageCallback, this);
     //detector = ORB::create(); // add parameters if needed
     cv::namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_odometry_node");
  node_class this_node;
  this_node.run_once();


  while (ros::ok()) {
      this_node.loop_function();
      ros::spinOnce();
  }

  return 0;
}
