#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
cout<<danish<<watup;
int main( int argc, char** argv )
{
	cout<<danish<<watup;
	char* imageName = argv[1];

	Mat image;
	image = imread( imageName, 1 );
	cout<<image;

	if( argc != 2 || !image.data )
	{
		printf( " No image data \n " );
		return -1;
	}
	namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
	  imshow( "Display window", image );          
}
