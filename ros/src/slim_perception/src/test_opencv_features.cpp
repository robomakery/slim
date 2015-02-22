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
  
  if (argc != 3) {
    std::cerr << "Wrong # of input arguments!" << std::endl;
    std::cerr << "Usage: " << argv[0] << " template.jpg target.jpg" << std::endl << std::endl;
    return -1;
  }

  cv::ORB extractor;
  std::vector<cv::KeyPoint> templateKeypoints, targetKeypoints;
  cv::Mat templateDescriptors, targetDescriptors;
  cv::Mat templateImageGray;
  cv::Mat targetImageGray;
  cv::Mat imageWithKeypoints, imageWithKeypointsResized;
  cv::Mat targetImageWithKeypoints, targetImageWithKeypointsResized;
  const float scale = 0.25f;

  // Read input template image 
  cv::Mat templateImage = cv::imread(argv[1]);

  // Read input target image to match template against
  cv::Mat targetImage = cv::imread(argv[2]);

  // Convert to grayscale
  cv::cvtColor(templateImage, templateImageGray, CV_BGR2GRAY);
  cv::cvtColor(targetImage, targetImageGray, CV_BGR2GRAY);

  // Create fake mask image
  cv::Mat maskImage = cv::Mat::ones(templateImage.size(), CV_8UC1);
  cv::Mat targetMaskImage = cv::Mat::ones(targetImage.size(), CV_8UC1);

  // Feature/descriptor extraction
  extractor(templateImageGray, maskImage, templateKeypoints, templateDescriptors);
  extractor(targetImageGray, targetMaskImage, targetKeypoints, targetDescriptors);


  // Feature/descriptor matching
  cv::BFMatcher matcher(cv::NORM_HAMMING, true);
  std::vector<cv::DMatch> matches;
  matcher.match(targetDescriptors, templateDescriptors, matches);

  // Draw matches
  cv::Mat imageMatches, imageMatchesResized;
  cv::drawMatches(targetImage, targetKeypoints, templateImage, templateKeypoints, matches, imageMatches);

  // Draw keypoints
//cv::drawKeypoints(templateImage, templateKeypoints, imageWithKeypoints);
//cv::drawKeypoints(targetImage, targetKeypoints, targetImageWithKeypoints);

  // Display results
//cv::resize(imageWithKeypoints, imageWithKeypointsResized,
//           cv::Size(scale * imageWithKeypoints.cols, scale * imageWithKeypoints.rows));
//cv::resize(targetImageWithKeypoints, targetImageWithKeypointsResized,
//           cv::Size(scale * targetImageWithKeypoints.cols, scale * targetImageWithKeypoints.rows));
  cv::resize(imageMatches, imageMatchesResized, 
             cv::Size(scale * imageMatches.cols, scale * imageMatches.rows));  
//cv::imshow("Template Keypoints", imageWithKeypointsResized);
//cv::imshow("Target Keypoints", targetImageWithKeypointsResized);
  cv::imshow("Matches", imageMatchesResized);
  cv::waitKey(0);

  return 0;
}
