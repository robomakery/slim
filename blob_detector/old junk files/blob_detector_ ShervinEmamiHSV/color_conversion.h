// color_conversion.h
//
#ifndef _COLOR_CONVERSION_H // must be unique name in the project
#define _COLOR_CONVERSION_H

// prototype declaration of the function in color_conversion.cpp
IplImage* convertImageRGBtoHSV(const IplImage *imageRGB);
IplImage* convertImageHSVtoRGB(const IplImage *imageHSV);

#endif 
