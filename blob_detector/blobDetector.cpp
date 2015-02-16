////////////////////////////////////////////////////////////////////////
// blobDetector.cpp
// Author Ian Hamilton
// imhamilton@users.noreply.github.com
//  2015 Feb 15th
//
// This file detects and identifies objects
// This will be incorporated into Team Robomakery's Slim robot for the
// Amazon Picking Challenge
//
///////////////////////////////////////////////////////////////////////

#include <cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <string>
//#include "color_conversion.h"

using namespace cv; 
 
IplImage* convertImageRGBtoHSV(const IplImage *imageRGB);

int main( int argc, char** argv )
{ 
  IplImage *imRGB;
  imRGB=cvLoadImage("./balls.jpg");
  if(imRGB == NULL)
	return 0;
  
  IplImage *imHSV;
  imHSV = convertImageRGBtoHSV(imRGB);	// Allocates a new HSV image.
  
  cvShowImage("RGB", imRGB);
  cvShowImage("HSV", imHSV);
  cvWaitKey();
  cvReleaseImage(&imHSV);		// Frees the new HSV image.
  cvReleaseImage(&imRGB);		// Frees the original RGB image.

 waitKey(0);

 return 0;
}


//convertImageRGBtoHSV function created by Shervin Emami 
//http://www.shervinemami.info/colorConversion.html
IplImage* convertImageRGBtoHSV(const IplImage *imageRGB)
{
	float fR, fG, fB;
	float fH, fS, fV;
	const float FLOAT_TO_BYTE = 255.0f;
	const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;

	// Create a blank HSV image
	IplImage *imageHSV = cvCreateImage(cvGetSize(imageRGB), 8, 3);
	if (!imageHSV || imageRGB->depth != 8 || imageRGB->nChannels != 3) {
		printf("ERROR in convertImageRGBtoHSV()! Bad input image.\n");
		exit(1);
	}

	int h = imageRGB->height;		// Pixel height.
	int w = imageRGB->width;		// Pixel width.
	int rowSizeRGB = imageRGB->widthStep;	// Size of row in bytes, including extra padding.
	char *imRGB = imageRGB->imageData;	// Pointer to the start of the image pixels.
	int rowSizeHSV = imageHSV->widthStep;	// Size of row in bytes, including extra padding.
	char *imHSV = imageHSV->imageData;	// Pointer to the start of the image pixels.
	for (int y=0; y<h; y++) {
		for (int x=0; x<w; x++) {
			// Get the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
			uchar *pRGB = (uchar*)(imRGB + y*rowSizeRGB + x*3);
			int bB = *(uchar*)(pRGB+0);	// Blue component
			int bG = *(uchar*)(pRGB+1);	// Green component
			int bR = *(uchar*)(pRGB+2);	// Red component

			// Convert from 8-bit integers to floats.
			fR = bR * BYTE_TO_FLOAT;
			fG = bG * BYTE_TO_FLOAT;
			fB = bB * BYTE_TO_FLOAT;

			// Convert from RGB to HSV, using float ranges 0.0 to 1.0.
			float fDelta;
			float fMin, fMax;
			int iMax;
			// Get the min and max, but use integer comparisons for slight speedup.
			if (bB < bG) {
				if (bB < bR) {
					fMin = fB;
					if (bR > bG) {
						iMax = bR;
						fMax = fR;
					}
					else {
						iMax = bG;
						fMax = fG;
					}
				}
				else {
					fMin = fR;
					fMax = fG;
					iMax = bG;
				}
			}
			else {
				if (bG < bR) {
					fMin = fG;
					if (bB > bR) {
						fMax = fB;
						iMax = bB;
					}
					else {
						fMax = fR;
						iMax = bR;
					}
				}
				else {
					fMin = fR;
					fMax = fB;
					iMax = bB;
				}
			}
			fDelta = fMax - fMin;
			fV = fMax;				// Value (Brightness).
			if (iMax != 0) {			// Make sure it's not pure black.
				fS = fDelta / fMax;		// Saturation.
				float ANGLE_TO_UNIT = 1.0f / (6.0f * fDelta);	// Make the Hues between 0.0 to 1.0 instead of 6.0
				if (iMax == bR) {		// between yellow and magenta.
					fH = (fG - fB) * ANGLE_TO_UNIT;
				}
				else if (iMax == bG) {		// between cyan and yellow.
					fH = (2.0f/6.0f) + ( fB - fR ) * ANGLE_TO_UNIT;
				}
				else {				// between magenta and cyan.
					fH = (4.0f/6.0f) + ( fR - fG ) * ANGLE_TO_UNIT;
				}
				// Wrap outlier Hues around the circle.
				if (fH < 0.0f)
					fH += 1.0f;
				if (fH >= 1.0f)
					fH -= 1.0f;
			}
			else {
				// color is pure Black.
				fS = 0;
				fH = 0;	// undefined hue
			}

			// Convert from floats to 8-bit integers.
			int bH = (int)(0.5f + fH * 255.0f);
			int bS = (int)(0.5f + fS * 255.0f);
			int bV = (int)(0.5f + fV * 255.0f);

			// Clip the values to make sure it fits within the 8bits.
			if (bH > 255)
				bH = 255;
			if (bH < 0)
				bH = 0;
			if (bS > 255)
				bS = 255;
			if (bS < 0)
				bS = 0;
			if (bV > 255)
				bV = 255;
			if (bV < 0)
				bV = 0;

			// Set the HSV pixel components.
			uchar *pHSV = (uchar*)(imHSV + y*rowSizeHSV + x*3);
			*(pHSV+0) = bH;		// H component
			*(pHSV+1) = bS;		// S component
			*(pHSV+2) = bV;		// V component
		}
	}
	return imageHSV;
}


