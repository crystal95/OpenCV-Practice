#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <numeric>     
using namespace cv;
using namespace std;
#define B 0.36571
#define cv 121.81265320213787
#define cu 589.1552373244099
#define focal 1994.694210980737
#define v_reso 1
#define v_range 10

typedef struct node{
vector<pair<float, pair<float,float> > > ans;
}node;

int main( int argc, char** argv )
{
    Mat image;
    Mat dis;
    int z_max=0, x_max=0;    
    image = imread(argv[1], CV_LOAD_IMAGE_UNCHANGED);   
    dis = imread(argv[2], CV_LOAD_IMAGE_UNCHANGED);   
    //cv::Mat tmp;
    Mat tmp= Mat(image.rows,image.cols,CV_32FC3);
    for(int h=0 ; h<image.rows ; h++)
    {
        for(int w=0; w<image.cols;w++)
        {
           Scalar intensity = dis.at<uchar>(h, w);
           if(intensity.val[0]!=0)
           {
                int Z=double(focal*B)/double(intensity.val[0]);
                tmp.at<Vec3f>( h,w )[0] = Z;
                tmp.at<Vec3f>( h,w )[1] = double(double(w-cu )*Z)/focal;
                tmp.at<Vec3f>( h,w )[2] =  double(double(cv-h )*Z)/focal;

               // cout<<int((tmp.at<Vec3f>( h,w )[0]*v_reso)/v_range)<<endl;

                if(z_max < int((tmp.at<Vec3f>( h,w )[0]*v_reso)/v_range))
                  z_max=int((tmp.at<Vec3f>( h,w )[0]*v_reso)/v_range);

                if(x_max<abs(int((tmp.at<Vec3f>( h,w )[1]*v_reso)/v_range)))
                    x_max=abs(int((tmp.at<Vec3f>( h,w )[1]*v_reso)/v_range));
            }
        }
    }
    //cout<<endl<<z_max<<endl;
    vector<vector <node> > sample(z_max+1, vector<node> (2*x_max+1));
    for(int h=0 ; h<image.rows ; h++)
    {
        for(int w=0; w<image.cols;w++)
        {
            int z,x;
            z= int ((tmp.at<Vec3f>( h,w )[0]*v_reso)/v_range);
            x = int ((tmp.at<Vec3f>( h,w )[1]*v_reso)/v_range);
           // cout<<z<<" "<<x<<endl;
            (sample[z][x+x_max].ans).push_back(make_pair(tmp.at<Vec3f>( h,w )[0],make_pair(tmp.at<Vec3f>( h,w )[1],tmp.at<Vec3f>( h,w )[2])));
             
        }
    }
    vector< vector<double> > final (z_max,vector<double> (2*x_max+1));
    for(int h=0 ; h<z_max; h++)
    {
        for(int w=0; w<2*x_max+1;w++)
        {
                double sum=0;
                int len = (sample[h][w].ans).size(),i;
                if(len>0)
                { 
                   

         /////////////////////////....mean calculation....////////////////////////////////////////////////

                    double sum;
                    vector<double> tmp;
                    vector<pair<float, pair<float , float> > > ::const_iterator it;
                    double min_elem=((sample[h][w].ans).begin())->first;
                    for(it=(sample[h][w].ans).begin();it!=(sample[h][w].ans).end();it++) 
                    {
                        tmp.push_back(it->first);
                        sum+=it->first;
                        if(min_elem>it->first)
                        {
                            min_elem=it->first;
                        }
                    }
                    double mean = sum / len;
            
     ///////////////////////.........standard deviation calculation ............................/////////////////////////////////////////////
                    
                    std::vector<double> diff((tmp).size());
                    std::transform((tmp).begin(), (tmp).end(), diff.begin(),std::bind2nd(std::minus<double>(), mean));
                    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
                    double std_dev = std::sqrt(sq_sum / tmp.size());

                                      
       /////////////// "final":it is a corresponding 2D vector whose values are calculated as :: Zi = max(min(zj ), mean(zj ) − σ)  
           
                    final[h][w] = max(min_elem,mean-std_dev);
                 
                }
           
        }

    }

    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", tmp );                   

    waitKey(0);                                          
    return 0;
}