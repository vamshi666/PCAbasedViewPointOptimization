/* Credits: */
// https://github.com/ros-planning/moveit_tutorials/blob/melodic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "servoing_franka/move_panda.h"

bool move_to_goal(servoing_franka::move_panda::Request &req, servoing_franka::move_panda::Response &res)
{
  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPlanningTime(10.0);
  move_group.allowReplanning(true);

  ROS_INFO_STREAM("Planner ID " << move_group.getPlannerId());

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  robot_state::RobotState start_state(*move_group.getCurrentState());

  move_group.setGoalTolerance(0.1);
  geometry_msgs::Pose target_pose;
  target_pose.orientation.x = req.pose.orientation.x;
  target_pose.orientation.y = req.pose.orientation.y;
  target_pose.orientation.z = req.pose.orientation.z;
  target_pose.orientation.w = req.pose.orientation.w;

  target_pose.position.x = req.pose.position.x;
  target_pose.position.y = req.pose.position.y;
  target_pose.position.z = req.pose.position.z;

  ROS_INFO_STREAM("Target pose is \n" << target_pose);

  move_group.setStartState(start_state);

  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();

  res.status = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_panda_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::ServiceServer service = nh.advertiseService("move_panda", move_to_goal);

  // Spin
  ros::waitForShutdown();
  return 0;
}