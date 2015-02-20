#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


class CloudVoxelizer
{
public:

  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  CloudVoxelizer(ros::NodeHandle &nh, 
                 const std::string &outputVoxelGridTopic, 
                 const double voxelSize = 0.01,
                 const bool latch = false);

  void voxelize(PointCloud::ConstPtr cloudPtr);
  void publish();
  
  PointCloud::Ptr getVoxelizedCloud();

private:
  ros::Publisher m_pub;
  pcl::VoxelGrid<PointCloud::PointType> m_voxelGrid;
  PointCloud::Ptr m_voxelizedCloud;
  double m_voxelSize;
};

//////////////////////////////////////////////////////////////////////////////
CloudVoxelizer::CloudVoxelizer(ros::NodeHandle &nh, 
                               const std::string &outputVoxelGridTopic, 
                               const double voxelSize /*= 0.01*/,
                               const bool latch /*= false*/)
{
  // Set up publisher for voxel grid 
  uint32_t queueSize = 1;
  m_pub = nh.advertise<PointCloud>(outputVoxelGridTopic, queueSize, latch);

  // Allocate output point cloud container
  m_voxelizedCloud.reset(new PointCloud);

  m_voxelSize = voxelSize;
}

//////////////////////////////////////////////////////////////////////////////
void CloudVoxelizer::voxelize(PointCloud::ConstPtr cloudPtr)
{
  m_voxelizedCloud->clear();

  // Filter input point cloud through voxel grid
  m_voxelGrid.setInputCloud(cloudPtr);
  m_voxelGrid.setLeafSize(m_voxelSize, m_voxelSize, m_voxelSize);  
  m_voxelGrid.filter(*m_voxelizedCloud);
}

//////////////////////////////////////////////////////////////////////////////
void CloudVoxelizer::publish()
{
  // Publish
  if (m_voxelizedCloud) {
    m_pub.publish(m_voxelizedCloud); 
  }
}

//////////////////////////////////////////////////////////////////////////////
CloudVoxelizer::PointCloud::Ptr CloudVoxelizer::getVoxelizedCloud()
{
  return m_voxelizedCloud;
}
