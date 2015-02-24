#include <cv.h>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/video/background_segm.hpp>
#include <cvblob.h>


using namespace cv;
using namespace cvb;

int const max_BINARY_value = 255;


void blobIan(char* imageName);
void checkArgs(

int main( int argc, char** argv )
{
 
  const char* imgName;
  if(argc > 1)
    imgName = argv[1];
  else
    imgName = "./inputFiles/balls.jpg";

 Mat imageOrig;
 
 imageOrig = imread( imgName, 1 );

 if( !imageOrig.data )
 {
   printf( " No image data \n " );
   return -1;
 }
 BinaryFilterBackground(imageOrig);
 waitKey(0);
 return 0;
}

void BinaryFilterBackground(Mat imageName)
{

 Mat HSV_image;
 cvtColor( image, HSV_image, CV_BGR2HSV );
 //imwrite( "./outputFiles/HSV_Image.jpg", HSV_image ); 
 //namedWindow( "HSV image", CV_WINDOW_AUTOSIZE );
 
 
 //split out hue and saturion values
 vector<Mat> hsv_planes;
 split( HSV_image, hsv_planes );
 //hsv_planes[0]; // H channel
 //hsv_planes[1]; // S channel
 //hsv_planes[2]; // V channel
  
 imshow( "H image",  hsv_planes[0] ); 
// imshow( "S image",  hsv_planes[1] ); 
 
 

 Mat h_BinaryBlue;
 inRange(hsv_planes[0], Scalar(101),Scalar(105),h_BinaryBlue);
 //imshow("h_BinaryBlue",h_BinaryBlue);
 Mat hBlueMask;
 //BackgroundSubtractor(h_BinaryBlue,hBlueMask,0);
 //imshow("hBlueMask",hBlueMask);

 //void BackgroundSubtractor::operator()(InputArray image, OutputArray fgmask, double learningRate=0)

  
 
 //printf( " H values \n ", );
 
 

 waitKey(0);


}//blobIan
