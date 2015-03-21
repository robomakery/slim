#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>

#include <boost/smart_ptr/scoped_ptr.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageFeatureMatcher
{
public:
  ImageFeatureMatcher(const cv::Mat trainingImage) :
//  m_featureDetector(400),
    m_trainingImage(trainingImage)
  {
    // Create fake mask image
    cv::Mat maskImage = cv::Mat::ones(trainingImage.size(), CV_8UC1);

    // Convert to grayscale
    cv::Mat trainingImageGray;
    cv::cvtColor(m_trainingImage, trainingImageGray, CV_BGR2GRAY);
    
    // Feature/descriptor extraction
//  m_featureDetector.detect(trainingImageGray, m_trainingKeypoints);
//  m_descriptorExtractor.compute(trainingImageGray, m_trainingKeypoints, m_trainingDescriptors);
    m_extractor(trainingImageGray, maskImage, m_trainingKeypoints, m_trainingDescriptors);
  }

  void match(const cv::Mat queryImage)
  {
    // Convert to grayscale
    cv::Mat queryImageGray;
    cv::cvtColor(queryImage, queryImageGray, CV_BGR2GRAY);

    // Create fake mask image
    cv::Mat queryMaskImage = cv::Mat::ones(queryImage.size(), CV_8UC1);

    // Feature/descriptor extraction
    std::vector<cv::KeyPoint> queryKeypoints;
    cv::Mat queryDescriptors;
//  m_featureDetector.detect(queryImageGray, queryKeypoints);
//  m_descriptorExtractor.compute(queryImageGray, queryKeypoints, queryDescriptors);
    m_extractor(queryImageGray, queryMaskImage, queryKeypoints, queryDescriptors);

    // Feature/descriptor matching
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;
    matcher.match(m_trainingDescriptors, queryDescriptors, matches);
//  cv::FlannBasedMatcher matcher;
//  std::vector<cv::DMatch> matches;
//  matcher.match(m_trainingDescriptors, queryDescriptors, matches);
//
    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < matches.size(); i++ )
    {
      double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }

//printf("-- Max dist : %f \n", max_dist );
//printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< cv::DMatch > good_matches;

    ROS_WARN_STREAM("m_trainingDescriptors.rows = " << m_trainingDescriptors.rows << ", m_trainingDescriptors.cols = " << m_trainingDescriptors.cols);
    ROS_WARN_STREAM("matches.size = " << matches.size());
    ROS_WARN_STREAM("min_dist = " << min_dist << ", max_dist = " << max_dist);

//    int len = matches.size();
//    for( int i = 0; i < len; i++ )
//    {
////    if( matches[i].distance <= std::max(2*min_dist, 0.5) )
//      if( matches[i].distance <= 1.1*min_dist)
//      { good_matches.push_back( matches[i]); }
//    }
//
//    ROS_WARN_STREAM("good_matches.size = " << good_matches.size());

////-- Draw only "good" matches
//cv::Mat img_matches;
//drawMatches( img_1, keypoints_1, img_2, keypoints_2,
//             good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//             vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


    // Draw matches
    cv::Mat imageMatches, imageMatchesResized;
//  cv::drawMatches(m_trainingImage, m_trainingKeypoints, queryImage, queryKeypoints,
//                  good_matches, imageMatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
//                  std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    cv::drawMatches(m_trainingImage, m_trainingKeypoints, queryImage, queryKeypoints, matches, imageMatches);

    // Resize image match
    const float scale = 1.0f;
    cv::resize(imageMatches, imageMatchesResized, 
               cv::Size(scale * imageMatches.cols, scale * imageMatches.rows));
    
    cv::imshow("Matches", imageMatchesResized);
    cv::waitKey(3);
  }

private:
  cv::ORB m_extractor;
//cv::SurfFeatureDetector m_featureDetector;
//cv::SurfDescriptorExtractor m_descriptorExtractor;
  cv::Mat m_trainingImage;
  std::vector<cv::KeyPoint> m_trainingKeypoints;
  cv::Mat m_trainingDescriptors;
};


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
//  cv::Mat trainingImage = cv::imread("/mnt/hgfs/llister/Devel/robomakery/berkeley/rgb_highres/templates/crayola.jpg");
//  cv::Mat trainingImage = cv::imread("/mnt/hgfs/llister/Devel/robomakery/berkeley/rgb_highres/templates/huckleberry.jpg");
    cv::Mat trainingImage = cv::imread("/mnt/hgfs/llister/Devel/robomakery/berkeley/rgb_highres/templates/feline.jpg");
    m_imageMatcherP.reset(new ImageFeatureMatcher(trainingImage));

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

//  // Draw an example circle on the video stream
//  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    m_imageMatcherP->match(cv_ptr->image);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

private:
  boost::scoped_ptr<ImageFeatureMatcher> m_imageMatcherP;
};

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "image_object_matcher_node");

  ImageConverter ic;

  ros::spin();

  return 0;
}
