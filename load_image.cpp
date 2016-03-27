//#include <cv.h>
//#include <highgui.h>
#include<stdio.h>
#include<iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
int main( int argc, char** argv )
{
	char* imageName = argv[1];

	Mat image;
	image = imread( imageName, 1 );

	if( argc != 2 || !image.data )
	{
		printf( " No image data \n " );
		return -1;
	}
	 //Mat R = Mat(3, 2, CV_8UC3);
	   //  randu(R, Scalar::all(0), Scalar::all(255));
	//cout<< image <<endl;

	/*Mat gray_image;
	cvtColor( image, gray_image, CV_BGR2GRAY );

	imwrite( "Gray_Image.jpg", gray_image );
*/
	namedWindow( imageName, CV_WINDOW_AUTOSIZE );
	namedWindow( "Gray image", CV_WINDOW_AUTOSIZE );

	imshow( imageName, image );
	//imshow( "Gray image", gray_image );

	waitKey(0);

	return 0;
}
