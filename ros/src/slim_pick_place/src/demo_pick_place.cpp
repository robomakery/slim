
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

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
  p.pose.position.x = -0.31;
  p.pose.position.y = 0.38;
  p.pose.position.z = 0.63;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  g.pre_grasp_approach.direction.vector.y = 1.0;
  g.pre_grasp_approach.direction.header.frame_id = "gripper_roll_link";
  g.pre_grasp_approach.min_distance = 0.2;
  g.pre_grasp_approach.desired_distance = 0.4;

  g.post_grasp_retreat.direction.header.frame_id = "base_link";
  g.post_grasp_retreat.direction.vector.y = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.25;

  g.pre_grasp_posture.joint_names.resize(1, "gripper_finger_pincher_joint");
  g.pre_grasp_posture.points.resize(1);
  g.pre_grasp_posture.points[0].positions.resize(1);
  g.pre_grasp_posture.points[0].positions[0] = 1;

  g.grasp_posture.joint_names.resize(1, "gripper_finger_pincher_joint");
  g.grasp_posture.points.resize(1);
  g.grasp_posture.points[0].positions.resize(1);
  g.grasp_posture.points[0].positions[0] = 0;

  g.allowed_touch_objects.resize(1);
  g.allowed_touch_objects[0] = "box";

  grasps.push_back(g);
  group.setSupportSurfaceName("shelf_bottom");
  group.pick("box", grasps);
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

  moveit::planning_interface::MoveGroup gantry_group("gantry");
  moveit::planning_interface::MoveGroup arm_group("arm");
  moveit::planning_interface::MoveGroup gripper_group("gripper_group");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  ros::WallDuration(1.0).sleep();

  // add shelf_bottom to the scene
  moveit_msgs::CollisionObject shelf_bottom;
  shelf_bottom.id = "shelf_bottom";
  shelf_bottom.header.stamp = ros::Time::now();
  shelf_bottom.header.frame_id = gantry_group.getPlanningFrame();

  // remove shelf_bottom
  std::vector<std::string> object_ids;
  object_ids.push_back(shelf_bottom.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  /* Define shelf_bottom's shape */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.435;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.01;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose bottom_pose;
  bottom_pose.orientation.w = 1.0;
  bottom_pose.position.x =  -0.3;
  bottom_pose.position.y = 0.4;
  bottom_pose.position.z =  0.6;

  shelf_bottom.primitives.push_back(primitive);
  shelf_bottom.primitive_poses.push_back(bottom_pose);
  shelf_bottom.operation = shelf_bottom.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(shelf_bottom);

  // add shelf_top
  moveit_msgs::CollisionObject shelf_top;
  shelf_top.id = "shelf_top";
  shelf_top.header.stamp = ros::Time::now();
  shelf_top.header.frame_id = gantry_group.getPlanningFrame();

  // remove shelf_top
  object_ids.clear();
  object_ids.push_back(shelf_top.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  /* Define shelf_top's shape */
  shape_msgs::SolidPrimitive top_primitive;
  top_primitive.type = primitive.BOX;
  top_primitive.dimensions.resize(3);
  top_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
  top_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.435;
  top_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.01;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose top_pose;
  top_pose.orientation.w = 1.0;
  top_pose.position.x =  -0.3;
  top_pose.position.y = 0.4;
  top_pose.position.z =  0.9;

  shelf_top.primitives.push_back(top_primitive);
  shelf_top.primitive_poses.push_back(top_pose);
  shelf_top.operation = shelf_top.ADD;

  collision_objects.push_back(shelf_top);

  // add shelf_left
  moveit_msgs::CollisionObject shelf_left;
  shelf_left.id = "shelf_left";
  shelf_left.header.stamp = ros::Time::now();
  shelf_left.header.frame_id = gantry_group.getPlanningFrame();

  // remove shelf_left
  object_ids.clear();
  object_ids.push_back(shelf_left.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  /* Define shelf_left's shape */
  shape_msgs::SolidPrimitive left_primitive;
  left_primitive.type = primitive.BOX;
  left_primitive.dimensions.resize(3);
  left_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.01;
  left_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.435;
  left_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose left_pose;
  left_pose.orientation.w = 1.0;
  left_pose.position.x =  -0.45;
  left_pose.position.y = 0.4;
  left_pose.position.z =  0.75;

  shelf_left.primitives.push_back(left_primitive);
  shelf_left.primitive_poses.push_back(left_pose);
  shelf_left.operation = shelf_left.ADD;

  collision_objects.push_back(shelf_left);
  // add shelf_right
  moveit_msgs::CollisionObject shelf_right;
  shelf_right.id = "shelf_right";
  shelf_right.header.stamp = ros::Time::now();
  shelf_right.header.frame_id = gantry_group.getPlanningFrame();

  // remove shelf_right
  object_ids.clear();
  object_ids.push_back(shelf_right.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  /* Define shelf_right's shape */
  shape_msgs::SolidPrimitive right_primitive;
  right_primitive.type = primitive.BOX;
  right_primitive.dimensions.resize(3);
  right_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.01;
  right_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.435;
  right_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose right_pose;
  right_pose.orientation.w = 1.0;
  right_pose.position.x =  -0.15;
  right_pose.position.y = 0.4;
  right_pose.position.z =  0.75;

  shelf_right.primitives.push_back(right_primitive);
  shelf_right.primitive_poses.push_back(right_pose);
  shelf_right.operation = shelf_right.ADD;

  collision_objects.push_back(shelf_right);

  // // add box to pick
  // moveit_msgs::CollisionObject box;
  // box.id = "box";
  // box.header.stamp = ros::Time::now();
  // box.header.frame_id = "base_link";

  // object_ids.clear();
  // object_ids.push_back(box.id);
  // planning_scene_interface.removeCollisionObjects(object_ids);

  // moveit_msgs::AttachedCollisionObject aco;
  // aco.object = box;
  // pub_aco.publish(aco);

  // shape_msgs::SolidPrimitive box_primitive;
  // box_primitive.type = primitive.BOX;
  // box_primitive.dimensions.resize(3);
  // box_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.05;
  // box_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.05;
  // box_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;

  // /* A pose for the box (specified relative to frame_id) */
  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x =  -0.31;
  // box_pose.position.y = 0.38;
  // box_pose.position.z =  0.63;

  // box.primitives.push_back(box_primitive);
  // box.primitive_poses.push_back(box_pose);
  // box.operation = box.ADD;

  // collision_objects.push_back(box);

  planning_scene_interface.addCollisionObjects(collision_objects);

  /* Sleep so we have time to see the objects in RViz */
  sleep(2.0);

  ROS_INFO("Picking scene ready.");

  ros::WallDuration(1.0).sleep();

  // try to move gantry
  // moveit::planning_interface::MoveGroup::Plan my_plan;
  // std::vector<double> group_variable_values;
  // group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  // group_variable_values[0] = 0.1;
  // group.setJointValueTarget(group_variable_values);
  // bool success = group.plan(my_plan);

  // ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
  // /* Sleep to give Rviz time to visualize the plan. */
  // sleep(5.0);
  // // move to the location
  // ROS_INFO("Moving to location");
  // group.move();
  // ROS_INFO("Done moving.");

  gripper_group.setNumPlanningAttempts(10);

  ROS_INFO("Moving to bin");
  gantry_group.setNamedTarget("demo_pick");
  gantry_group.move();
  sleep(2.0);

  // ROS_INFO("Opening Gripper");
  // gripper_group.setNamedTarget("open");
  // gripper_group.move();
  // sleep(2.0);
  
  ROS_INFO("Pre-grasp pose");
  arm_group.setNamedTarget("cobra");
  arm_group.move();
  
  sleep(2.0);

  ROS_INFO("Grasp pose");
  arm_group.setNamedTarget("cobra2");
  arm_group.move();
  
  sleep(2.0);

  // ROS_INFO("Close Gripper");
  // gripper_group.setNamedTarget("closed");
  // gripper_group.move();
  
  sleep(10.0);

  ROS_INFO("Post-grasp");
  arm_group.setNamedTarget("cobra");
  arm_group.move();

  sleep(2.0);

  ROS_INFO("Reset Gantry");
  gantry_group.setNamedTarget("default");
  gantry_group.move();

  sleep(2.0);

  ROS_INFO("Place");
  arm_group.setNamedTarget("bowl");
  arm_group.move();

  sleep(2.0);

  ROS_INFO("Drop");
  gripper_group.setNamedTarget("open");
  gripper_group.move();

  sleep(2.0);

  ROS_INFO("Moving to bin");
  gantry_group.setNamedTarget("demo2_pick");
  gantry_group.move();
  sleep(2.0);

  ROS_INFO("Pre-grasp pose");
  arm_group.setNamedTarget("cobra");
  arm_group.move();
  
  sleep(2.0);

  ROS_INFO("Grasp pose");
  arm_group.setNamedTarget("cobra2");
  arm_group.move();
  
  sleep(2.0);

  // ROS_INFO("Close Gripper");
  // gripper_group.setNamedTarget("closed");
  // gripper_group.move();
  
  sleep(10.0);

  ROS_INFO("Post-grasp");
  arm_group.setNamedTarget("cobra");
  arm_group.move();

  sleep(2.0);

  ROS_INFO("Reset Gantry");
  gantry_group.setNamedTarget("default");
  gantry_group.move();

  sleep(2.0);

  ROS_INFO("Place");
  arm_group.setNamedTarget("bowl");
  arm_group.move();

  sleep(2.0);

  ROS_INFO("Drop");
  gripper_group.setNamedTarget("open");
  gripper_group.move();

  sleep(2.0);
  

  // pick(arm_group);
  // sleep(1.0);
  // ros::WallDuration(1.0).sleep();
  // place(arm_group);

  ros::shutdown();
  return 0;
}
