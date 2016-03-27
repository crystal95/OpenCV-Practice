#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include<sstream>
#include<iostream>
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include<sstream>
#include <opencv/cv.h>
#include "detectobs.cpp"
#include <fstream>
#include <iostream>
using namespace std;
int rc_old = 0;
int rc = 0;
int H = 150;
cv::Mat imageGray;
cv::Mat imageColor;

  int i=0;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
++i;  
  try  
   { 
   //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //string ssave1 = "/home/iiith/Desktop/new/" + ss.str()+ ".jpg";
    imageGray = cv_bridge::toCvShare(msg, "mono8")->image;
    cv::imshow("view", cv_bridge::toCvShare(msg, "mono8")->image);       
    imageColor = cv_bridge::toCvShare(msg, "bgr8")->image;
    //cv::imshow("imageColor", cv_bridge::toCvShare(msg, "bgr8")->image);    
    //cv::imwrite(ssave1,cv_bridge::toCvShare(msg, "bgr8")->image);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_subsc");
  cv::namedWindow("view");
  ros::NodeHandle nh;
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  stringstream ss;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Int16>("midpoint", 10);
  image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 10, imageCallback);
  std_msgs::Int16 msg; 
  int move  = 0;
  while(ros::ok())
  {
  ros::spinOnce();
 if(i>0)
 {
    ss<<i;
    
  }
  }
  //cv::destroyWindow("view");
}
