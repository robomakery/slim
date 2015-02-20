#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::visualization::CloudViewer::MonochromeCloud PointCloud;


void pointCloudToVoxelGrid(const PointCloud &inCloud, PointCloud &outCloud)
{

  const float voxel_grid_size = 0.05f; //0.005f;
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  PointCloud::ConstPtr inCloudConstPtr(new PointCloud(inCloud));
  vox_grid.setInputCloud (inCloudConstPtr);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
  vox_grid.filter (*tempCloud);
  outCloud = *tempCloud;
}

void foo ()
{
  PointCloud cloud;

  // Read in pcd file
  std::string filename = "/mnt/hgfs/llister/Devel/robomakery/data/woodenShelf/scene_with_shelf.pcd";
  pcl::io::loadPCDFile(filename, cloud);

  // Create const pointer (memcopy) for visualization
  PointCloud::ConstPtr cloudPtr(new PointCloud(cloud));

  // Downsample input point cloud to make a voxel grid
//PointCloud voxelGrid;
//pointCloudToVoxelGrid(cloud, voxelGrid);
//PointCloud::ConstPtr voxelGridPtr(new PointCloud(voxelGrid));

  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(cloudPtr);
//viewer.showCloud(voxelGridPtr);
  while (!viewer.wasStopped ())
  {
  }
}

int main(int argc, char **argv)
{
  foo();

  return 0;
}
