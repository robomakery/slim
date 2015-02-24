#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <highgui.h>

using namespace cv;


int main( int argc, char** argv )
{
 char* imageName = argv[1];

 //Mat imRGB; 
 IplImage *imBGR;
 imBGR = imread( imageName, 1 );

 if( argc != 2 || !imBGR.data )
 {
   printf( " No image data \n " );
   return -1;
 }

 //Mat imHSV;
 IplImage *imHSV;
 imHSV = convertRGBtoHSV(imBGR);	// Allocates a new HSV image.
 
 imwrite( "../../images/HSVimage.jpg", imHSV );

 namedWindow( "HSV image", CV_WINDOW_AUTOSIZE );
 imshow( "HSV image", imHSV );

 waitKey(0);

 return 0;
   
 

}
