

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;



void lane_tracking(Mat *input)
{
	 Mat track(*input.rows,*input.cols, CV_8UC1); 
for(int i = 0; i < track.rows; i++)
{
    for(int j = 0; j < track.cols; j++)
    {
       double pixel=track.at<uchar>(i, j)=1;
       cout<<pixel<<endl;     
    }

}

}


int main( int argc, char** argv )
{
     Mat image;
    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);  
    namedWindow( "Display window", WINDOW_AUTOSIZE );
   	imshow( "Display window", image );    
    waitKey(0);  

    lane_tracking(&image);


    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", image );    
    waitKey(0);  

                                           
    return 0;
}