#include <cv.h>
#include <highgui.h>
#include <cvblob.h>

using namespace cv;
using namespace cvb;


int main( int argc, char** argv )
{
 
  const char* imageName;
  if(argc > 1)
    imageName = argv[1];
  else
    imageName = "./inputFiles/balls.jpg";

 Mat image;
 
 image = imread( imageName, 1 );

 if( !image.data )
 {
   printf( " No image data \n " );
   return -1;
 }

 Mat HSV_image;
 cvtColor( image, HSV_image, CV_BGR2HSV );
 
 imwrite( "./outputFiles/HSV_Image.jpg", HSV_image );
 namedWindow( "HSV image", CV_WINDOW_AUTOSIZE );
 imshow( "HSV image", HSV_image );

 waitKey(0);

 return 0;
}
