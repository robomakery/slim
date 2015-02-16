#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string>

using namespace cv;
 



int main( int argc, char** argv )
{ 
 //hardcoded filename
 //char imgNameMatrix[] = "balls.jpg";
 //std::cout << "The file is " << imgNameMatrix;

 char* imageName = argv[1];
 //char* imageName; 	//delete this if taking arguments in main
 //imageName= &imgNameMatrix; //delete this if taking arguments in main

 Mat RGB_mat;
 RGB_mat = imread( imageName, 1 );
 
 IplImage *iplRGB;
 iplRGB = CvLoadImage(imageName);
 
 

 // Mat HSV_mat;
// cvtColor( RGB_mat, HSV_mat, CV_BGR2HSV );
 IplImage *iplHSV;
 iplHSV = convertRGBtoHSV(iplRGB);	// Allocates a new HSV image.
 
 //convert ipl back to mat
 Mat HSV_mat(iplHSV);
 

 imwrite( "../../images/Gray_Image.jpg", HSV_mat );

 namedWindow( imageName, CV_WINDOW_AUTOSIZE );
 namedWindow( "Gray image", CV_WINDOW_AUTOSIZE );

 imshow( imageName, RGB_mat );
 imshow( "HSV image", HSV_mat );

 waitKey(0);

 cvReleaseImage(&imHSV);		// Only frees one of the new images, not both!
 cvReleaseImage(&imRGB);		// Frees the original RGB image.

 return 0;
}
