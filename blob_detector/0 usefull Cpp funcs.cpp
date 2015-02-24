

imwrite( "./outputFiles/HSV_Image.jpg", HSV_image );
imwrite( "./outputFiles/HSV_Image.jpg", HSV_image ); 
 namedWindow( "HSV image", CV_WINDOW_AUTOSIZE );
 void BackgroundSubtractor::operator()(InputArray image, OutputArray fgmask, double learningRate=0)

threshold( src_gray, dst, threshold_value, max_BINARY_value,threshold_type );
threshold_type should be zero for binary.

double threshold(InputArray src, OutputArray dst, double thresh, double maxval, int type)
void inRange(InputArray src, InputArray lowerb, InputArray upperb, OutputArray dst)

Mat output;
inRange(src, Scalar(140),Scalar(160),output_array);
imshow("processed",output);
waitKey();


Matrix values

cv::Mat m = ...;

for(int r = 0; r < m.rows; ++r)
{
  for(int c = 0; c < m.cols; ++c)
  {
    char byte = m.at<unsigned char>(r, c);
    ...
  }
}
printf( " Hues \n ", hsv_planes.at<unsigned char>(22,22) );//creates error
