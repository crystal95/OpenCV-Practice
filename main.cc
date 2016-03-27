/**
 * \file main.cc
 * \author Mohamed Aly <malaa@caltech.edu>
 * \date Wed Oct 6, 2010
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "main.hh"
#include "cmdline.h"
#include "LaneDetector.hh"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <opencv/cv.h>
#include <cv.h>
#include <highgui.h>
//#include "~/Desktop/catkinws_10/src/roslanes/"
#include <ros/console.h>
 //#include <foo_msgs/Foo.h>
int i=0,ans=0;
using namespace std;
using namespace cv;
int d=0;
cv::Mat raw_mat1;
int flag=0,cnt1=0;
float mycount=0;
sensor_msgs::ImagePtr mymsg;
 cv::Mat image(10,1,CV_32FC3, Scalar(0,0,0));
 int argC;char **argV;  
//cv::Mat raw_matB;
//cv::Mat mat1;
//using namespace cv;
// Useful message macro

#define MSG(fmt, ...) \
  (fprintf(stdout, "%s:%d msg   " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? 0 : 0)

// Useful error macro
#define ERROR(fmt, ...) \
  (fprintf(stderr, "%s:%d error " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : -1)
    

    std_msgs::Float32MultiArray arrx;
    std_msgs::Float32MultiArray arry;

      //foo_msgs::Foo arrx ;


namespace LaneDetector
{

    /**
     * This function reads lines from the input file into a vector of strings
     *
     * \param filename the input file name
     * \param lines the output vector of lines
     */
  bool ReadLines(const char* filename, vector<string> *lines)
  {
    // make sure it's not NULL
    if (!lines)
      return false;
    // resize
    lines->clear();

    ifstream  file;
    file.open(filename, ifstream::in);
    char buf[5000];
    // read lines and process
    while (file.getline(buf, 5000))
    {
      string str(buf);
      lines->push_back(str);
    }
    // close
    file.close();
    return true;
  }


    /**
     * This function processes an input image and detects lanes/stoplines
     * based on the passed in command line arguments
     *
     * \param filename the input file name
     * \param cameraInfo the camera calibration info
     * \param lanesConf the lane detection settings
     * \param stoplinesConf the stop line detection settings
     * \param options the command line arguments
     * \param outputFile the output file stream to write output lanes to
     * \param index the image index (used for saving output files)
     * \param elapsedTime if NOT NULL, it is accumulated with clock ticks for
     *        the detection operation MSG("RASHID SODHI IS A GOOD BOY");
     */
     void ProcessImage( CameraInfo& cameraInfo,
                  LaneDetectorConf& lanesConf, LaneDetectorConf& stoplinesConf,
                  gengetopt_args_info& options, ofstream* outputFile,
                  int index, clock_t *elapsedTime,cv::Mat raw_mat2)  //defined in IPM.hh
    {
      

      IplImage* im;
      im = cvCreateImage(cvSize(raw_mat2.cols,raw_mat2.rows),8,3);
      IplImage ipltemp = raw_mat2;
      cvCopy(&ipltemp,im);
      CvMat temp;
      //usleep(30000000);
      if (!im)///////////////////////////////////////////////
      {        //////////////////////////////////////////////////
          // print error and exit
          cout << "ERROR: Capture is null!\n";///////////////////
          return ; ////////////////////////////////////////////
      }    ///
      cvGetMat(im, &temp);
      CvMat *schannel_mat;
      CvMat* tchannelImage = cvCreateMat(im->height, im->width, INT_MAT_TYPE);
      cvSplit(&temp, tchannelImage, NULL, NULL, NULL);
      // convert to float
      CvMat *channelImage= cvCreateMat(im->height, im->width, FLOAT_MAT_TYPE);
      cvConvertScale(tchannelImage, channelImage, 1./255);
      vector<FLOAT> lineScores, splineScores;
      vector<Line> lanes;
      vector<Spline> splines;

      clock_t startTime = clock();
      MSG ("**********procesing images*********");
      mcvGetLanes(channelImage, &temp, &lanes, &lineScores, &splines, &splineScores,   //1st image=5
                  &cameraInfo, &lanesConf, NULL);  
      cout<<"MY NAME IS DANISH"<<endl;
CvMat *imDisplay = cvCloneMat(&temp);

          /*for(int i=0; i<splines.size(); i++)
          {
           
    

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           // if (splines[i].color == LINE_COLOR_YELLOW)
            //{

       CvMat *pixels = mcvGetBezierSplinePixels(splines[i], .05,
                                           cvSize(imDisplay->width, imDisplay->height),
                                           false);
            if (pixels)
            {

             
              int x1=(cvGetReal2D(pixels, 0, 0));
              int y1= cvGetReal2D(pixels, 0, 1);
              int x2=(cvGetReal2D(pixels, 19, 0));
              int y2=cvGetReal2D(pixels, 19, 1);
              arrx.data.push_back(x1);
              arrx.data.push_back(y1);
              arrx.data.push_back(x2);
              arrx.data.push_back(y2);
              flag=1;
              
              cvCircle( imDisplay, cvPoint(x1,y1) , 8, CV_RGB(255, 0, 0),-1 );
              cvCircle( imDisplay, cvPoint(x2,y2) , 8, CV_RGB(255, 0, 0),-1 );

           // }
          }
        }*/
                    ///defined in lanedetector.cc

                   if(flag==0)
                   {
                    printf("MY SIZE IS %lld\n", (long long int)lanes.size());
                    
                  /*  for(int i=0; i<lanes.size(); i++){
                   arrx.data.push_back((int)lanes[i].startPoint.x);
             arrx.data.push_back((int)lanes[i].startPoint.y);
              arrx.data.push_back((int)lanes[i].endPoint.x);
               arrx.data.push_back((int)lanes[i].endPoint.y);
               flag=1;*/
             //}
             //  printf("%d\n", (int)lanes[i].startPoint.y);
             }

      ROS_INFO("processsed***********8");
      clock_t endTime = clock();
      MSG("Found %d lanes in %f msec", splines.size(),
          static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC * 1000.);
      // update elapsed time
      if (elapsedTime)
        (*elapsedTime) += endTime - startTime;

 
      // save results?
      if (options.save_lanes_flag && outputFile && outputFile->is_open())
      {

        (*outputFile) << "frame#" << setw(8) << setfill('0') << index <<
          " has " << splines.size() << " splines" << endl;
        for (int i=0; i<splines.size(); ++i)
        {
          (*outputFile) << "\tspline#" << i+1 << " has " <<
            splines[i].degree+1 << " points and score " <<
            splineScores[i] << endl;
          for (int j=0; j<=splines[i].degree; ++j)
            (*outputFile) << "\t\t" <<
              splines[i].points[j].x << ", " <<
              splines[i].points[j].y << endl;
        }
      }

      // show or save


      if (options.show_flag || options.save_images_flag)
      {

        // show detected lanes
        ///cout<<"breaking here******************";
       // CvMat *imDisplay = cvCloneMat(&temp);

        // convert to BGR
    //     cvCvtColor(raw_mat, imDisplay, CV_RGB2BGR);
        if (lanesConf.ransacLine && !lanesConf.ransacSpline)
          {
 
            for(int i=0; i<lanes.size(); i++)
           {
            
            mcvDrawLine(imDisplay, lanes[i], CV_RGB(0,125,0), 3);
            //arrx.data.push_back((int)lanes[i].startPoint.x);
             //arrx.data.push_back((int)lanes[i].startPoint.y);
              //arrx.data.push_back((int)lanes[i].endPoint.x);
               //arrx.data.push_back((int)lanes[i].endPoint.y);
               //printf("%d\n", (int)lanes[i].startPoint.y);



          }
        }

        // print lanes
        if (lanesConf.ransacSpline)
        {
            MSG("RASHID SODHI IS A GOOD BOY");
            printf(" NO OF SPLINES ARE %d\n", int(splines.size()) );
          for(int i=0; i<splines.size(); i++)
          {
           //splines.size();
    

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            if (splines[i].color == LINE_COLOR_YELLOW)
            {
         //MSG("RASHID");////////////////////
               CvMat *pixels = mcvGetBezierSplinePixels(splines[i], .05,
                                           cvSize(imDisplay->width, imDisplay->height),
                                           false);
            if (pixels )
            {

                MSG("RASHID SODHI IS HERE");

              //MSG("RASHID SODHI");
              int x1=(cvGetReal2D(pixels, 0, 0));
              int y1= cvGetReal2D(pixels, 0, 1);
              int x2=(cvGetReal2D(pixels, 19, 0));
              int y2=cvGetReal2D(pixels, 19, 1);
              arrx.data.push_back(x1);
              arrx.data.push_back(y1);
              arrx.data.push_back(x2);
              arrx.data.push_back(y2);
              //flag=1; cout<<endl<<endl<<"lane number "<<lanecount<<endl;
      
              //cvCircle( imDisplay, cvPoint(x1,y1) , 8, CV_RGB(255, 0, 0),-1 ); ///REPRESNET a POINTmove
              //cvCircle( imDisplay, cvPoint(x2,y2) , 8, CV_RGB(255, 0, 0),-1 );

            }

              mcvDrawSpline1(imDisplay, splines[i], CV_RGB(255,255,0), 3);
            }/////***************calling here..
            /////////////////////////// //////////////////////////////////////////////////////////////////////////////////////////////////////////
            else
              {
                 MSG("IIIIIIIIIIIIIIIIIMMMMMMMMMHERRRRRRRR");
                

            CvMat *pixels = mcvGetBezierSplinePixels(splines[i], .05,
                                           cvSize(imDisplay->width, imDisplay->height),
                                           false);
            if (pixels )
            {
               
              
              
              int x1=(cvGetReal2D(pixels, 19, 0));
              int y1=cvGetReal2D(pixels, 19, 1);
              if(1==1)
              {
                for(int it=0;it<=19;it++)
                {
                  
                  int x=(cvGetReal2D(pixels, it, 0));
                  int y= cvGetReal2D(pixels, it, 1);

                  arrx.data.push_back(x);
                  arrx.data.push_back(y);
                  /* arrx.data.push_back(x1);
                  arrx.data.push_back(y1);
                  arrx.data.push_back(x2);
                  arrx.data.push_back(y2);
                  */
                  cvCircle( imDisplay, cvPoint(x,y) , 8, CV_RGB(255, 0, 0),-1 );

               
                 //   image.at<cv::Vec3f>(it/2,0)[0] = x;
                   // image.at<cv::Vec3f>(it/2,0)[1] = y;
                    //image.at<cv::Vec3f>(it/2,0)[2] = 786 ;
                    //cout<<image.at<cv::Vec3f>(j,0)<<endl;     
                  //cvCircle( imDisplay, cvPoint(x2,y2) , 8, CV_RGB(255, 0, 0),-1 );
                
                }
                arrx.data.push_back(-1);
                arrx.data.push_back(-1);
                 mymsg = cv_bridge::CvImage(std_msgs::Header(), "32FC3", image).toImageMsg();

              }
              else
              {
                for(int it=0;it<=19;it++)
                {
                  int x=(cvGetReal2D(pixels, it, 0));
                  int y= cvGetReal2D(pixels, it, 1);
                  
                  arry.data.push_back(x);
                  arry.data.push_back(y);
                  /* arrx.data.push_back(x1);
                  arrx.data.push_back(y1);
                  arrx.data.push_back(x2);
                  arrx.data.push_back(y2);
                  */
                  cvCircle( imDisplay, cvPoint(x,y) , 8, CV_RGB(0, 0, 255),-1 );
                  
                  //cvCircle( imDisplay, cvPoint(x2,y2) , 8, CV_RGB(255, 0, 0),-1 );
                
                }

              }
              
              
              
            }
                mcvDrawSpline1(imDisplay, splines[i], CV_RGB(0,255,0), 3);
              }
            // print numbers?
            if (options.show_lane_numbers_flag)
            {
              char str[256];
              sprintf(str, "%d", i);
              mcvDrawText(imDisplay, str,
                          cvPointFrom32f(splines[i].points[splines[i].degree]),
                                          1, CV_RGB(0, 0, 255));
            }
          }
        }
        // show?
        if (options.show_flag)
        {
          // set the wait value
          int wait = options.step_flag ? 0 : options.wait_arg;
          // show image with detected lanes
          SHOW_IMAGE(imDisplay, "Detected Lanes", wait);
    /*    cv::Mat myfinal; 
  myfinal = cv::Mat(imDisplay);
std::stringstream ss1;                           ///////////////////////////////for image saving
ss1 << cnt1;
std::string str1 = ss1.str();
std::string final1=str1+".jpg";
 cv::imwrite(final1,myfinal); 
    cnt1++;*/

    
  



        }
        cvReleaseMat(&imDisplay);
      }


     //** cvReleaseMat(&temp);left_image
      //cvReleaseMat(**tempB);
       //**cvReleaseMat(tchannelImage);
       //**cvReleaseImage(&im);
    }



    int Process(int argc, char** argv ,cv::Mat raw_mat2)
    {
      // parse the command line paramters
      

      gengetopt_args_info options;
      if (cmdline_parser (argc, argv,  &options) < 0)
        return -1;

      // read the camera configurations
      CameraInfo cameraInfo;  //defined in IPM.hh
      mcvInitCameraInfo(options.camera_conf_arg, &cameraInfo); //defined in inverseperspectivemaping.cc
      MSG("Loaded camera file");

      // read the configurations
      LaneDetectorConf lanesConf, stoplinesConf; //struct defenition found in Landedetector.hh(whic has params stored in Lanes.conf)
      if (!options.no_lanes_flag)
      {
        mcvInitLaneDetectorConf(options.lanes_conf_arg, &lanesConf); //defined in lanedetector.cc
        MSG("Loaded lanes config file");
      }
      if (!options.no_stoplines_flag)
      {
        mcvInitLaneDetectorConf(options.stoplines_conf_arg, &stoplinesConf);
        MSG("Loaded stop lines config file");
      }

      // set debug to true
      if (options.debug_flag)
         DEBUG_LINES = 1;

      // process a single image
      //if (options.image_file_given)
      //{
        // elapsed time
        clock_t elapsed = 0;
        //ROS_INFO("Calling ProcessImages**************");
        ProcessImage( cameraInfo, lanesConf, stoplinesConf,
                      options, NULL, elapsed, 0 , raw_mat2);
        double elapsedTime = static_cast<double>(elapsed) / CLOCKS_PER_SEC;
        MSG("Total time %f secs for 1 image = %f Hz", elapsedTime,
            1. / elapsedTime);

      return 0;
    }
    

} // namespace LaneDetector
 

        // show detected lanes
        ///cout<<"breaking here******************";
       // CvMat *imDisplay =
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //ROS_INFO("xcjkhkjzcbhkjxb");
   i++;
  try  
     { 
        arrx.data.clear();
       arry.data.clear();
       
      raw_mat1 = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::imshow("view", raw_mat1);  
      if (!raw_mat1.data)///////////////////////////////////////////////
          {        //////////////////////////////////////////////////
              // print error and exit
              cout << "EMPTY IMAGE**********************************************123\n";///////////////////
               ////////////////////////////////////////////
          }  
  
       //ROS_INFO("Calling Process**************");
       LaneDetector::Process(argC,argV, raw_mat1);
     
      
   
      
   } 
    catch (cv_bridge::Exception& raw_mat1)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    
    
}
clock_t start = clock();
// using LaneDetector::Process;

// main entry point
int main(int argc, char** argv)
{
  argC=argc;argV=argv;
  //clock_t start = clock();
  arrx.data.clear();
  arry.data.clear();
  ros::init(argc, argv, "my_subsc");
  cv::namedWindow("view");
  ros::NodeHandle nh;
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  stringstream ss;
  //ros::Publisher chatter_pub = nh.advertise<std_msgs::Int16>("midpoint", 10);
  ROS_DEBUG("Hello %s", "World");
  //ros::Publisher pubx = nh.advertise<std_msgs::Int32MultiArray>("/array_basic", 1);
   ros::Publisher pubx = nh.advertise<std_msgs::Float32MultiArray>("/array_basic", 1);
   ros::Publisher puby = nh.advertise<std_msgs::Float32MultiArray>("/array_basic1", 1);
//image_transport::Subscriber sub = it.subscribe("/camera/left/image_raw", 10, imageCallback);
  image_transport::Subscriber sub = it.subscribe("/danish", 10, imageCallback);

  image_transport::Publisher pubimg = it.advertise("/myimage", 1);
  std_msgs::Int16 msg; 
  int move  = 0;//image_rect_color   ///kitti_stereo/left/image_rect
 // /camera/left/image_raw
  //LaneDetector::Process(argc,argv, raw_mat1);  
  while(ros::ok())
  {       ros::spinOnce();


      pubimg.publish(mymsg);
      pubx.publish(arrx);
      puby.publish(arry);
      arrx.data.clear();
      arry.data.clear();
     
  }

 clock_t end = clock();
  double ans =static_cast<double>((end - start) / CLOCKS_PER_SEC * 1000.);
mycount++;

}
