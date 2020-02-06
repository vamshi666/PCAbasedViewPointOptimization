/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */
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


bool move_to_goal(servoing_franka::move_panda::Request &req, servoing_franka::move_panda::Response &res){

    // ros::AsyncSpinner spinner(1); spinner.start();

    static const std::string PLANNING_GROUP = "arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPlanningTime(10.0);
    move_group.allowReplanning(true);

    ROS_INFO_STREAM("Planner ID "<<move_group.getPlannerId());

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    // namespace rvt = rviz_visual_tools;
    // moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    // visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    // visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    // text_pose.translation().z() = 1.75;
    // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    // visual_tools.trigger();

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));

    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.

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

    ROS_INFO_STREAM("Target pose is \n"<<target_pose);
    
    // start_state.setFromIK(joint_model_group, target_pose);
    move_group.setStartState(start_state);

    move_group.setPoseTarget(target_pose);
    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    // visual_tools.publishAxisLabeled(target_pose, "target pose");
    // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Moving to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^
    //
    // Moving to a pose goal is similar to the step above
    // except we now use the move() function. Note that
    // the pose goal we had set earlier is still active
    // and so the robot will try to move to that goal. We will
    // not use that function in this tutorial since it is
    // a blocking function and requires a controller to be active
    // and report success on execution of a trajectory.

    /* Uncomment below line when working with a real robot */
    move_group.move();

    res.status = true;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_panda_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); spinner.start();

    ros::ServiceServer service = nh.advertiseService("move_panda", move_to_goal);

    // Spin
    // ros::spin();
    ros::waitForShutdown();
    // ros::spin();

    return 0;
}