#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub;

void callback(const PointCloud::ConstPtr& msg)
{
  ROS_INFO_STREAM("Point Cloud Time = " << msg->header.stamp);

  if (msg->header.stamp == 1417906971003448) {
    ROS_INFO("<<< Got the right stamp. Writing to pcd.");

    pcl::io::savePCDFileASCII("/mnt/hgfs/llister/Devel/robomakery/data/woodenShelf/scene_with_shelf.pcd", *msg);
  }
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "slim_perception");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  int rate;
  string topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(40));
  private_node_handle_.param("topic", topic, string("/camera/depth/points"));

  // Create a publisher
  pub = n.advertise<PointCloud> ("points2", 1);

  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  ros::Subscriber sub_message = n.subscribe<PointCloud>(topic.c_str(), 1, callback);

  ros::spin();

  return 0;
}
