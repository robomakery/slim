#include <cv.h>
#include <highgui.h>

using namespace cv;

int main( int argc, char** argv )
{
 char* imageName;

 if( argv != NULL)
 {
  printf( " argv !=NULL");
  
  imageName = argv[1];
 }
 else
 { 
   printf( " Default file ./balls.jpg \n " );

   char temp[] = "./balls.jpg";
   printf( " Temp " );
   imageName = temp;
   
 }

 Mat image;
 
 image = imread( imageName, 1 );

 if( argc != 2 || !image.data )
 {
   printf( " No image data \n " );
   return -1;
 }

 Mat HSV_image;
 cvtColor( image, HSV_image, CV_BGR2HSV );

 imwrite( "../../images/HSV_Image.jpg", HSV_image );
 //imwrite( "./HSV_Image.jpg", HSV_image );

 //namedWindow( imageName, CV_WINDOW_AUTOSIZE );
 namedWindow( "HSV image", CV_WINDOW_AUTOSIZE );

 imshow( imageName, image );
 imshow( "HSV image", HSV_image );

 waitKey(0);

 return 0;
}
