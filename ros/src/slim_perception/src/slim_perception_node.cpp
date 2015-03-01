#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//#include <boost/foreach.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

//using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//typedef sensor_msgs::PointCloud2 PointCloud;

ros::Publisher pubPassthrough;

double passThroughLimitMin = 0.0f;
double passThroughLimitMax = 2.0f;

void callback(const PointCloud::ConstPtr& msg)
{
  // Initialize point cloud for range-based pass through filtering
  PointCloud::Ptr cloudPassThroughFiltered(new PointCloud);

  PointCloud::Ptr cloudExtracted(new PointCloud);
//PointCloud::Ptr filteredCloud(new PointCloud);


  // Pass Through Filter object.
  pcl::PassThrough<pcl::PointXYZ> filter;
  filter.setInputCloud(msg);
  // Filter out all points with Z values not in the [min-max] range.
  filter.setFilterFieldName("z");
  filter.setFilterLimits(passThroughLimitMin, passThroughLimitMax);
  
  filter.filter(*cloudPassThroughFiltered);

  // kd-tree object for searches.
//pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

  // Filter object.
//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
//filter.setInputCloud(msg);
//// Set number of neighbors to consider to 50.
//filter.setMeanK(50);
//// Set standard deviation multiplier to 1.
//// Points with a distance larger than 1 standard deviation of the mean distance will be outliers.
//filter.setStddevMulThresh(1.0);

//// Filter object.
//pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
//// Every point must have 10 neighbors within 15cm, or it will be removed.
//filter.setRadiusSearch(0.15);
//filter.setMinNeighborsInRadius(10);
//
//filter.filter(*filteredCloud);

  #if 0
  // Plane segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.03);
  segmentation.setInputCloud(msg);

  // Object for storing the indices.
  pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices);

  segmentation.segment(*pointIndices, *coefficients);

  // Object for extracting points from a list of indices.
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(msg);
  extract.setIndices(pointIndices);
  // We will extract the points that are NOT indexed (the ones that are not in a plane).
  extract.setNegative(true);
  extract.filter(*cloudExtracted);
#endif

//// Euclidean clustering object.
//kdtree->setInputCloud(cloudExtracted);
//pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
//// Set cluster tolerance to 2cm (small values may cause objects to be divided
//// in several clusters, whereas big values may join objects in a same cluster).
//clustering.setClusterTolerance(0.02);
//// Set the minimum and maximum number of points that a cluster can have.
//clustering.setMinClusterSize(100);
//clustering.setMaxClusterSize(25000);
//clustering.setSearchMethod(kdtree);
//clustering.setInputCloud(cloudExtracted);
//std::vector<pcl::PointIndices> clusters;
//clustering.extract(clusters);

//  // For every cluster...
//  int currentClusterNum = 1;
//  for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
//  {
//    // ...add all its points to a new cloud...
//    PointCloud::Ptr cluster(new PointCloud);
//    for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
//      cluster->points.push_back(cloudExtracted->points[*point]);
//    cluster->width = cluster->points.size();
//    cluster->height = 1;
//    cluster->is_dense = true;
//
//    // ...and save it to disk.
//    if (cluster->points.size() <= 0)
//      break;
////  std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
////  std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
////  pcl::io::savePCDFileASCII(fileName, *cluster);
//
//    pubPassthrough.publish(cluster);
//
//    currentClusterNum++;
//  }

  if (pubPassthrough.getNumSubscribers() > 0) {
    pubPassthrough.publish(cloudPassThroughFiltered);
  }
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "slim_perception");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  int rate;
  std::string inputCloudTopic;
  std::string passthroughOutputCloudTopic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(40));
  private_node_handle_.param("topic", inputCloudTopic, std::string("/camera/depth/points"));
  private_node_handle_.param("limit_min", passThroughLimitMin, 0.0);
  private_node_handle_.param("limit_max", passThroughLimitMax, 2.0);
  private_node_handle_.param("passthrough_output_cloud_topic", passthroughOutputCloudTopic, std::string(""));

  if (!passthroughOutputCloudTopic.empty()) {
    // Create a publisher
    pubPassthrough = n.advertise<PointCloud> (passthroughOutputCloudTopic, 1);
  }

  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  ros::Subscriber sub_message = n.subscribe<PointCloud>(inputCloudTopic.c_str(), 1, callback);

  ros::spin();

  return 0;
}
