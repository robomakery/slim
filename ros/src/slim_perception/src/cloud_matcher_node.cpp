#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d_omp.h>

#include <boost/scoped_ptr.hpp>

#include "slim_perception/CloudVoxelizer.h"

boost::scoped_ptr<CloudVoxelizer> cloudVoxelizer;
boost::scoped_ptr<CloudVoxelizer> cadModelVoxelizer;

typedef CloudVoxelizer::PointCloud PointCloud;

//////////////////////////////////////////////////////////////////////////////
void callback(const PointCloud::ConstPtr msg)
{
  // Downsample incoming scene point cloud to voxel grid
  cloudVoxelizer->voxelize(msg);
  cloudVoxelizer->publish();

  // Get pointer to CAD model voxel grid
  PointCloud::Ptr cadModelVoxelGrid = cadModelVoxelizer->getVoxelizedCloud();

//pcl::NormalEstimationOMP<pcl::PointNormal,pcl::PointNormal> nest;
//nest.setRadiusSearch(0.01);
//nest.setInputCloud(*msg);
//nest.compute(*msg);
}


//////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "cloud_matcher");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  std::string topic;
  std::string modelFilePath;
  double voxelSize;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("topic", topic, std::string("/camera/depth/points"));
  private_node_handle_.param("model_file_path", modelFilePath, std::string("/mnt/hgfs/llister/Devel/robomakery/data/woodenShelf/Model.ply"));
  private_node_handle_.param("voxel_size", voxelSize,  0.01);

  // Initialize CloudVoxelizer object for depth camera point clouds
//cloudVoxelizer.reset(new CloudVoxelizer(n, std::string("/camera/depth/voxelized_points"), voxelSize));

  // Initialize CloudVoxelizer object for CAD model
  // Enable latching b/c model doesn't change and new subscribers can get data later on
  cadModelVoxelizer.reset(new CloudVoxelizer(n, std::string("/cad_model/voxelized_points"), voxelSize, true));

  // Load CAD model to point cloud
  pcl::PLYReader modelReader;
  PointCloud modelPointCloud;
  modelReader.read(modelFilePath, modelPointCloud);
  modelPointCloud.header.frame_id = "/camera_depth_frame";
  PointCloud::ConstPtr cloudPtr(new PointCloud(modelPointCloud));


  // Voxelize CAD model point cloud
  cadModelVoxelizer->voxelize(cloudPtr);
  cadModelVoxelizer->publish();  // Publish once (latched). May need to put this after a spin call.
  
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
//ros::Subscriber sub_message = n.subscribe<PointCloud>(topic.c_str(), 1, callback);

  ros::spin();

  return 0;
}
