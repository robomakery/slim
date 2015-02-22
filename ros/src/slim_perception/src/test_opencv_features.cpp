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
  std::cout << "Training Image Matching Test" << std::endl;
  
  if (argc != 3) {
    std::cerr << "Wrong # of input arguments!" << std::endl;
    std::cerr << "Usage: " << argv[0] << " training.jpg query.jpg" << std::endl << std::endl;
    return -1;
  }

  cv::ORB extractor;
  std::vector<cv::KeyPoint> trainingKeypoints, queryKeypoints;
  cv::Mat trainingDescriptors, queryDescriptors;
  cv::Mat trainingImageGray;
  cv::Mat queryImageGray;
  cv::Mat imageWithKeypoints, imageWithKeypointsResized;
  cv::Mat queryImageWithKeypoints, queryImageWithKeypointsResized;
  const float scale = 0.25f;

  // Read input training image 
  cv::Mat trainingImage = cv::imread(argv[1]);

  // Read input query image to match training against
  cv::Mat queryImage = cv::imread(argv[2]);

  // Convert to grayscale
  cv::cvtColor(trainingImage, trainingImageGray, CV_BGR2GRAY);
  cv::cvtColor(queryImage, queryImageGray, CV_BGR2GRAY);

  // Create fake mask image
  cv::Mat maskImage = cv::Mat::ones(trainingImage.size(), CV_8UC1);
  cv::Mat queryMaskImage = cv::Mat::ones(queryImage.size(), CV_8UC1);

  // Feature/descriptor extraction
  extractor(trainingImageGray, maskImage, trainingKeypoints, trainingDescriptors);
  extractor(queryImageGray, queryMaskImage, queryKeypoints, queryDescriptors);

  // Feature/descriptor matching
  cv::BFMatcher matcher(cv::NORM_HAMMING, true);
  std::vector<cv::DMatch> matches;
  matcher.match(queryDescriptors, trainingDescriptors, matches);

  // Draw matches
  cv::Mat imageMatches, imageMatchesResized;
  cv::drawMatches(queryImage, queryKeypoints, trainingImage, trainingKeypoints, matches, imageMatches);

  // Draw keypoints
//cv::drawKeypoints(trainingImage, trainingKeypoints, imageWithKeypoints);
//cv::drawKeypoints(queryImage, queryKeypoints, queryImageWithKeypoints);

  // Display results
//cv::resize(imageWithKeypoints, imageWithKeypointsResized,
//           cv::Size(scale * imageWithKeypoints.cols, scale * imageWithKeypoints.rows));
//cv::resize(queryImageWithKeypoints, queryImageWithKeypointsResized,
//           cv::Size(scale * queryImageWithKeypoints.cols, scale * queryImageWithKeypoints.rows));
  cv::resize(imageMatches, imageMatchesResized, 
             cv::Size(scale * imageMatches.cols, scale * imageMatches.rows));  
//cv::imshow("Training Keypoints", imageWithKeypointsResized);
//cv::imshow("Query Keypoints", queryImageWithKeypointsResized);
  cv::imshow("Matches", imageMatchesResized);
  cv::waitKey(0);

  return 0;
}

