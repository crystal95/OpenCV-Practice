#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>

using namespace std;
using namespace cv;


Mat & ScanImageAndReduce(Mat &I,const unchar* const table)
{
	CV_Assert(I.depth()!=sizeof(uchar));

	int channels = I.channels();
	int nRows = I.rows;
	int nCols = I.cols * channels;

	if (I.isContinuous())
	{
		nCols *= nRows;
		nRows = 1;
	}
	int i,j;
	uchar* p;
	for( i = 0; i < nRows; ++i)
	{
		p = I.ptr<uchar>(i);
		for ( j = 0; j < nCols; ++j)
		{
			p[j] = table[p[j]];
		}
	}
	return I;
}


int main(int argc, char * argv[])
{
	 uchar table[256];
	if (argc < 3)
	{
		cout << "Not enough parameters" << endl;
		return -1;
	}
	Mat I, J;
	if( argc == 4 && !strcmp(argv[3],"G") )
		I = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	else
		I = imread(argv[1], CV_LOAD_IMAGE_COLOR);   


	if (!I.data)
	{
		cout << "The image" << argv[1] << " could not be loaded." << endl;
		return -1;
	}
	int dividewith=0;
	stringstream s;
	s << argv[2];
	s>>dividewith ;
	if(!s || ! dividewith)
	{
		cout<<"Invalid";
		return -1;
	}
	uchar table[256];
	for(i=0;i<256;i++)
		table[i]=uchar((i/dividewith)*dividewith);
	const int times = 100;
	double t;
	t = (double)getTickCount();
	for (int i = 0; i < times; ++i)
	{
		cv::Mat clone_i = I.clone();
		J = ScanImageAndReduceC(clone_i, table);
	}

	t = 1000*((double)getTickCount() - t)/getTickFrequency();
	t /= times;

	cout << "Time of reducing with the C operator [] (averaged for "
		<< times << " runs): " << t << " milliseconds."<< endl

}

