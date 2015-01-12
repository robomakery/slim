
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

#include <moveit_msgs/DisplayTrajectory.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

void pick(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::Grasp> grasps;
}

void place(moveit::planning_interface::MoveGroup &group)
{
}

int main(int argc, char **argv)
{
  ROS_INFO_STREAM_NAMED("main","Starting Slim Demo Pick Place");

  ros::init (argc, argv, "slim_demo_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  moveit::planning_interface::MoveGroup group("gantry");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  ros::WallDuration(1.0).sleep();

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  /* The id of the object is used to identify it. */
  collision_object.id = "box1";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.6;
  box_pose.position.y = -0.4;
  box_pose.position.z =  1.2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  ROS_INFO("Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  /* Sleep so we have time to see the object in RViz */
  sleep(2.0);

  // try to move gantry
  moveit::planning_interface::MoveGroup::Plan my_plan;
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  group_variable_values[0] = 0.1;
  group.setJointValueTarget(group_variable_values);
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);
  // move to the location
  ROS_INFO("Moving to location");
  group.move();
  ROS_INFO("Done moving.");

  pick(group);

  ros::WallDuration(1.0).sleep();

  place(group);

  ros::shutdown();
  return 0;
}
