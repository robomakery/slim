#include <cv.h>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/video/background_segm.hpp>
#include <cvblob.h>


using namespace cv;
using namespace cvb;

//int const max_BINARY_value = 255;


int main( int argc, char** argv )
{
 
  const char* imgName;
  if(argc > 1)
    imgName = argv[1];
  else
    imgName = "./inputFiles/balls.jpg";

 Mat image;
 
 image = imread( imgName, 1 );

 if( !image.data )
 {
   printf( " No image data \n " );
   return -1;
 }
 Mat HSV_image;
 cvtColor( image, HSV_image, CV_BGR2HSV );
 //imwrite( "./outputFiles/HSV_image.jpg", HSV_image ); 
  
 namedWindow( "HSV image", CV_WINDOW_AUTOSIZE );
 imshow( "./outputFiles/HSV_image.jpg", HSV_image ); 
 
 //split out hue and saturion values
 vector<Mat> hsv_planes;
 split( HSV_image, hsv_planes );
 //hsv_planes[0]; // H channel
 //hsv_planes[1]; // S channel
 //hsv_planes[2]; // V channel
  

 //imwrite( "./outputFiles/H_meta.jpg", hsv_planes[0]  ); 
 //imwrite( "./outputFiles/S_meta.jpg", hsv_planes[1]  );

 Mat hBinaryBlue;
 inRange(hsv_planes[0], Scalar(101),Scalar(105),hBinaryBlue);
 imshow("hBinaryBlue",hBinaryBlue);
 
 //Mat hBlueMask;
 //BackgroundSubtractor(hBinaryBlue,hBlueMask,0);
 //imshow("hBlueMask",hBlueMask);

 //void BackgroundSubtractor::operator()(InputArray image, OutputArray fgmask, double learningRate=0)

 //Use cvBlob library to get location and dimensions on blob
 // library available at http://cvblob.googlecode.com
 // cvblobs Struct takes a binary and original image and creates a blob list
 //  CvBlobs blobs;
  // IplImage iplHBinBlue = hBinaryBlue; 
  // IplImage iplimg = image; 
//   IplImage* iplHBinBlue = new IplImage(hBinaryBlue);
//IplImage* iplimg = new IplImage(image);
  // unsigned int result=cvLabel(iplHBinBlue, iplimg, blobs);
   
 
 
//Release iplImage????HOW?
 waitKey(0);
 return 0;
}


