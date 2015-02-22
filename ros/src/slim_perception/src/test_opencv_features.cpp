/**
 * File: test_opencv_features.cpp 
 * Author: Levi Lister 
 */

// System
#include <iostream>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

int main(int argc, char **argv)
{
  std::cout << "Template Image Matching Test" << std::endl;
  
  if (argc != 2) {
    std::cerr << "Wrong # of input arguments!" << std::endl;
    return -1;
  }

  // Read input template image 
  cv::Mat templateImage = cv::imread(argv[1]);

  // Convert template image to grayscale
  cv::Mat templateImageGray;
  cv::cvtColor(templateImage, templateImageGray, CV_BGR2GRAY);

  // Create fake mask image
  cv::Mat maskImage = cv::Mat::ones(templateImage.size(), CV_8UC1);

  // Print image image
  std::cout << "Template Image (Gray):\n";
  std::cout << "  Size: " << templateImageGray.cols << " x " << templateImageGray.rows << std::endl;
  std::cout << "  Depth: " << templateImageGray.depth() << ", Channels: " << templateImageGray.channels() << ", Type: " << templateImageGray.type() << std::endl;

  // OpenCV ORB feature extractor
  cv::ORB orbFeatureExtractor;
  std::vector<cv::KeyPoint> keypoints;
  orbFeatureExtractor(templateImageGray, maskImage, keypoints);

  // Draw keypoints
  cv::Mat imageWithKeypoints, imageWithKeypointsResized;
  cv::drawKeypoints(templateImage, keypoints, imageWithKeypoints);

  // Display keypoints
  const float scale = 0.5f;
  cv::resize(imageWithKeypoints, imageWithKeypointsResized, cv::Size(scale * imageWithKeypoints.cols, scale * imageWithKeypoints.rows));
  cv::imshow("Keypoints", imageWithKeypointsResized);
  cv::waitKey(0);

  return 0;
}
