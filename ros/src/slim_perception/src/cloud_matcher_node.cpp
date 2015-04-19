#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/recognition/line_rgbd.h>

#include <boost/scoped_ptr.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> ColorPointCloud;

//////////////////////////////////////////////////////////////////////////////
/// Detector class for matching a CAD model to point clouds
//////////////////////////////////////////////////////////////////////////////
class DetectorLineRGBD
{
 public:
   DetectorLineRGBD(ros::NodeHandle nh, const std::string &topic) 
   {
     // Create a subscriber.
     // Name the topic, message queue, callback function with class name, and object containing callback function.
     m_sub = nh.subscribe(topic.c_str(), 1, &DetectorLineRGBD::callback, this);
   }

   //////////////////////////////////////////////////////////////////////////////
   void loadTemplateFromModel(const std::string modelFilePath)
   {
     // Load CAD model to point cloud
     pcl::PLYReader modelReader;
     ColorPointCloud modelPointCloud;
     modelReader.read(modelFilePath, modelPointCloud);
     modelPointCloud.header.frame_id = "/camera_depth_frame";

     // Set up mask of template object to train
     pcl::MaskMap mask_map_xyz(modelPointCloud.width, modelPointCloud.height);
     pcl::MaskMap mask_map_rgb(modelPointCloud.width, modelPointCloud.height); //0, 0);
     pcl::RegionXY region_xy;
     region_xy.width = modelPointCloud.width;
     region_xy.height = modelPointCloud.height;

     // Load point cloud into LineRGBD object
     const std::size_t object_id = 0;
     m_line_rgbd.createAndAddTemplate(modelPointCloud, object_id, mask_map_xyz, mask_map_rgb, region_xy);
   }
   
 private:
   pcl::LineRGBD<PointT> m_line_rgbd;
   ros::Subscriber m_sub;

   //////////////////////////////////////////////////////////////////////////////
   void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
   {
     ROS_INFO_STREAM("Processing point cloud at time: " << cloud->header.stamp);
   }
};


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

  ROS_INFO("[Cloud Matcher] Starting point cloud matcher node.");

  ROS_INFO("[Cloud Matcher] Initializing detector");

  // Initialize detector object
  DetectorLineRGBD detector(n, topic);

  ROS_INFO("[Cloud Matcher] Loading template model");

  // Load template model
  detector.loadTemplateFromModel(modelFilePath);

  ROS_INFO("[Cloud Matcher] Done loading template model");

  // Initialize CloudVoxelizer object for depth camera point clouds
//cloudVoxelizer.reset(new CloudVoxelizer(n, std::string("/camera/depth/voxelized_points"), voxelSize));

  // Initialize CloudVoxelizer object for CAD model
  // Enable latching b/c model doesn't change and new subscribers can get data later on
//cadModelVoxelizer.reset(new CloudVoxelizer(n, std::string("/cad_model/voxelized_points"), voxelSize, true));


//// Voxelize CAD model point cloud
//cadModelVoxelizer->voxelize(cloudPtr);
//cadModelVoxelizer->publish();  // Publish once (latched). May need to put this after a spin call.
  
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
//ros::Subscriber sub_message = n.subscribe<ColorPointCloud>(topic.c_str(), 1, &DetectorLineRGBD::callback, &detector);

  ros::spin();

  return 0;
}
