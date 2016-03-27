

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "iostream"
using namespace cv;
using namespace std;



void lane_tracking(Mat *input)
{
    int i,j;
    cout<<input->rows<<" "<<input->cols<<endl;

	 Mat track(input->rows,input->cols, CV_8UC1); 
     //cout<<input->cols
     cout<<track.rows<<" "<<track.cols<<endl;
for( i = 0; i < input->rows; i++)
{
    for(j = 0; j < input->cols; j++)
    {
      track.at<uchar>(i,j)=input->at<uchar>(i, j);
       //cout<<pixel<<endl;     
    }

}
 namedWindow( "Display window2", WINDOW_AUTOSIZE );
    imshow( "Display window2", track );    
waitKey(0);
}


int main( int argc, char** argv )
{
     Mat image;
    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);  
   
    cout<<image.rows<<" "<<image.cols<<image.channels()<<endl;
     cvtColor(image,image, CV_BGR2GRAY);
         cout<<image.rows<<" "<<image.cols<<image.channels()<<endl;


    lane_tracking(&image);


   // namedWindow( "Display window2", WINDOW_AUTOSIZE );
    //imshow( "Display window2", image );    
    //waitKey(0);  

                                           
    return 0;
}