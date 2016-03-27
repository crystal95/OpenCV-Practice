#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
int main( int argc, char** argv )
{

	cout<<" fuck off " <<endl; 
	char* imageName = argv[1];

	Mat image;
	image = imread( imageName, 1 );

//	cout<<image ;
	if( argc != 2 || !image.data )
	{
		printf( " No image data \n " );
		return -1;
	}


	namedWindow( imageName, CV_WINDOW_AUTOSIZE );

	imshow( imageName, image );
		waitKey(0);

	return 0;
}
