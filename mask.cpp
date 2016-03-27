//#include <cv.h>
//#include <highgui.h>
#include<stdio.h>
#include<iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

void Sharpen(const Mat& myimage, Mat& Result)
{
    CV_Assert(myimage.depth() == CV_8U);  // accept only uchar images

    Result.create(myimage.size(), myimage.type());
    const int nChannels = myimage.channels();

    for(int j = 1; j < myimage.rows - 1; ++j)
    {
        const uchar* previous = myimage.ptr<uchar>(j - 1);
        const uchar* current  = myimage.ptr<uchar>(j    );
        const uchar* next     = myimage.ptr<uchar>(j + 1);

        uchar* output = Result.ptr<uchar>(j);

        for(int i = nChannels; i < nChannels * (myimage.cols - 1); ++i)
        {
            *output++ = saturate_cast<uchar>(5 * current[i]
                         -current[i - nChannels] - current[i + nChannels] - previous[i] - next[i]);
        }
    }

    Result.row(0).setTo(Scalar(0));
    Result.row(Result.rows - 1).setTo(Scalar(0));
    Result.col(0).setTo(Scalar(0));
    Result.col(Result.cols - 1).setTo(Scalar(0));
}

int main( int argc, char** argv )
{
	char* imageName = argv[1];

	Mat myimage;
	myimage = imread( imageName, 1 );

	if( argc != 2 || !myimage.data )
	{
		printf( " No image data \n " );
		return -1;
	}
	namedWindow( "myimageName", CV_WINDOW_AUTOSIZE );
	imshow( "myimageName", myimage );


	Mat Result;

	Sharpen(myimage,Result);

	namedWindow( "resultimage", CV_WINDOW_AUTOSIZE );
	imshow( "resultimage", Result);

	
	waitKey(0);

	return 0;
}
