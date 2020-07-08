// stl
#include <thread>

// ros
#include "ros/ros.h"
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

// Moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// services' headers
#include <servoing_franka/move_panda.h>
#include <servoing_franka/segment_service.h>
#include <servoing_franka/pose_service.h>
#include <servoing_franka/fusecloud_service.h>
#include <servoing_franka/graspquality_service.h>
#include <servoing_franka/computepca_service.h>

using namespace std::chrono_literals;  // Needs c++14

#define NORMAL_ANGLE_TH 20

// States
enum class State
{
  INIT,
  GO_HOME,
  GRAB_POINTCLOUD,
  SEGMENT,
  FUSE,
  CHECK_GRASPQUALITY,
  COMPUTE_PCA,
  COMPUTE_NEXTACTION,
  GOTONEXTPOSE,
  DONE
};

// Initializations
State state = State::INIT;
pcl::PCLPointCloud2 current_cloud;
pcl::PCLPointCloud2 previous_cloud;
geometry_msgs::Pose initial_goal_pose, current_pose, next_pose;
pcl::PCLPointCloud2 cloud_object;
pcl::PCLPointCloud2 fused_cloud_object;
geometry_msgs::Point object_centroid;
geometry_msgs::Vector3 major_vector, middle_vector, minor_vector;

// get quaternion pose from transformation
geometry_msgs::PoseStamped get_pose_from_transform(tf::StampedTransform tf)
{
  // clumsy conversions--points, vectors and quaternions are different data types in tf vs geometry_msgs
  geometry_msgs::PoseStamped stPose;
  geometry_msgs::Quaternion quat;  // geometry_msgs object for quaternion
  tf::Quaternion tfQuat;      // tf library object for quaternion
  tfQuat = tf.getRotation();  // member fnc to extract the quaternion from a transform
  quat.x = tfQuat.x();        // copy the data from tf-style quaternion to geometry_msgs-style quaternion
  quat.y = tfQuat.y();
  quat.z = tfQuat.z();
  quat.w = tfQuat.w();
  stPose.pose.orientation = quat;  // set the orientation of our PoseStamped object from result

  // now do the same for the origin--equivalently, vector from parent to child frame
  tf::Vector3 tfVec;  // tf-library type
  geometry_msgs::Point pt;  // equivalent geometry_msgs type
  tfVec = tf.getOrigin();  // extract the vector from parent to child from transform
  pt.x = tfVec.getX();  // copy the components into geometry_msgs type
  pt.y = tfVec.getY();
  pt.z = tfVec.getZ();
  stPose.pose.position = pt;  // and use this compatible type to set the position of the PoseStamped
  stPose.header.frame_id = tf.frame_id_;  // the pose is expressed w/rt this reference frame
  stPose.header.stamp = tf.stamp_;  // preserve the time stamp of the original transform
  return stPose;
}

// get current camera co-ordintates based on end effector position
geometry_msgs::Pose getCurentCameraCoordinates()
{
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("camera_link");

  /* Print end-effector pose. Remember that this is in the model frame */
  // ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  // ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(end_effector_state, pose);

  // ROS_INFO_STREAM("pose: \n" << pose << "\n");

  tf::TransformListener listener;
  tf::StampedTransform transform;
  geometry_msgs::PoseStamped pose_stamped;
  while (ros::ok())
  {
    try
    {
      listener.waitForTransform("world", "panda_link8", ros::Time(0), ros::Duration(5.0), ros::Duration(2));
      listener.lookupTransform("world", "panda_link8", ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }

  pose_stamped = get_pose_from_transform(transform);
  pose = pose_stamped.pose;

  ROS_INFO_STREAM("Transform from panda_link0 to camera_link: \n" << pose_stamped);

  return pose;
}

// Change state from INIT to GO_HOME only when pointcloud data is being published
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  if (state == State::INIT)
  {
    state = State::GO_HOME;
    pcl_conversions::toPCL(*cloud_msg, current_cloud);
    ROS_INFO("STATE -  GO HOME");
  }

  if (state == State::GRAB_POINTCLOUD)
  {
    pcl_conversions::toPCL(*cloud_msg, current_cloud);
    // std::cout<<"Current state is GRABPOINTCLOUD"<<std::endl;
    ROS_INFO("STATE -  GRAB POINTCLOUD");
    state = State::SEGMENT;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Exploration_state_machine");

  ros::NodeHandle nh;

  // Create a ROS subscriber to check if the input pointcloud is being published at all
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  ros::ServiceClient move_panda_client = nh.serviceClient<servoing_franka::move_panda>("move_panda");
  ros::ServiceClient segment_service_client = nh.serviceClient<servoing_franka::segment_service>("segment_service");
  ros::ServiceClient fusecloud_service_client =
      nh.serviceClient<servoing_franka::fusecloud_service>("fusecloud_service");
  ros::ServiceClient graspquality_service_client =
      nh.serviceClient<servoing_franka::graspquality_service>("graspquality_service");
  ros::ServiceClient computepca_service_client =
      nh.serviceClient<servoing_franka::computepca_service>("computepca_service");
  ros::ServiceClient pose_service_client = nh.serviceClient<servoing_franka::pose_service>("pose_service");

  ros::Rate loop_rate(10);
  bool done = true;
  while (ros::ok())
  {
    ros::spinOnce();
    if (state == State::GO_HOME)
    {
      // ros::ServiceClient client = nh.serviceClient<servoing_franka::move_panda>("move_panda");
      current_pose = getCurentCameraCoordinates();

      ROS_INFO_STREAM("CURRENT POSE is" << current_pose << "\n");

      // all_zeroes    pose:
      //     x: 0.146414
      //     y: 3.79507e-06
      //     z: 0.940634
      // orientation:
      //     x: 0.99833
      //     y: 6.45262e-05
      //     z: 0.0577688
      //     w: -3.55704e-06

      // HOME POSITION
      //       HOME position:                      //    LEFT POSITION
      //     x: 0.3326                            //     x: 0.224146
      //     y: 0.00421936                        //     y: -5.76873e-06
      //     z: 2.00111                           //     z: 0.744779
      //   orientation:                           //   orientation:
      //     x: 0.937607                          //     x: -0.359446
      //     y: 0.0108788                         //     y: -0.000277897
      //     z: 0.34733                           //     z: 0.933166
      //     w: -0.0116478                        //     w: -9.08005e-05

      initial_goal_pose.position.x = 0.146414;
      initial_goal_pose.position.y = 3.79507e-06;
      initial_goal_pose.position.z = 0.940634;

      Eigen::Quaterniond resultant_quaternion;
      Eigen::Vector3d object_position(0.4d, 0.0d, 0.01d);
      Eigen::Vector3d current_position(initial_goal_pose.position.x, initial_goal_pose.position.y,
                                       initial_goal_pose.position.z);
      Eigen::Vector3d final_vector(object_position - current_position);
      Eigen::Vector3d rz = final_vector.normalized();
      Eigen::Vector3d ybasis(0, 1, 0);
      Eigen::Vector3d rx = rz.cross(ybasis).normalized();
      Eigen::Vector3d ry = rz.cross(rx).normalized();
      Eigen::Matrix3d orientation;
      orientation << rx, ry, rz;
      resultant_quaternion = orientation;
      tf::quaternionEigenToMsg(resultant_quaternion, initial_goal_pose.orientation);

      std::cout << "Orientation matrix" << orientation << std::endl;
      ROS_INFO_STREAM("HOME orientation" << initial_goal_pose.orientation);

      servoing_franka::move_panda move_panda_srv;
      move_panda_srv.request.pose = initial_goal_pose;
      if (move_panda_client.call(move_panda_srv))
      {
        bool done = false;
        while (!done)
        {
          done = move_panda_srv.response.status;
        }
        ROS_INFO("Panda moved to HOME");
        state = State::GRAB_POINTCLOUD;  // Change state
      }
      else
      {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
      }
    }
    geometry_msgs::Pose current_pose = getCurentCameraCoordinates();

    ROS_INFO_STREAM("AFTER achieiving goal..  POSE is" << current_pose << "\n");
    if (state == State::SEGMENT)
    {
      ros::spinOnce();
      ROS_INFO("State - SEGMENT");
      servoing_franka::segment_service segment_service_srv;
      pcl_conversions::fromPCL(current_cloud, segment_service_srv.request.in_cloud);

      if (segment_service_client.call(segment_service_srv))
      {
        bool done = false;
        while (!done)
        {
          pcl_conversions::toPCL(segment_service_srv.response.out_cloud, cloud_object);
          done = segment_service_srv.response.status;
        }
        ROS_INFO("BOWL SEGMENTATION DONE");
        state = State::FUSE;  // Change state
      }
      else
      {
        ROS_ERROR("Failed to call segment_service");
        return 1;
      }
      sensor_msgs::PointCloud2 cloud_object_ros;
      pcl_conversions::fromPCL(cloud_object, cloud_object_ros);
      ROS_INFO_STREAM("BOWL_OBJECT:\n" << cloud_object_ros);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromPCLPointCloud2(cloud_object,
                              *point_cloud_ptr);  //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>

      pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
      viewer.showCloud(point_cloud_ptr);
      while (!viewer.wasStopped())
      {
      }
    }

    if (state == State::FUSE)
    {
      ROS_INFO("CURRENT STATE : FUSE");
      servoing_franka::fusecloud_service fusecloud_service_srv;
      pcl_conversions::fromPCL(cloud_object, fusecloud_service_srv.request.current_cloud);
      if (fusecloud_service_client.call(fusecloud_service_srv))
      {
        bool done = false;
        while (!done)
        {
          done = fusecloud_service_srv.response.status;
        }
        pcl_conversions::toPCL(fusecloud_service_srv.response.fused_cloud, fused_cloud_object);
        ROS_INFO("BOWL SEGMENTATION DONE");
        state = State::CHECK_GRASPQUALITY;  // Change state
      }
      else
      {
        ROS_ERROR("Failed to call fusecloud_service");
        return 1;
      }
      sensor_msgs::PointCloud2 fused_cloud_object_ros;
      pcl_conversions::fromPCL(fused_cloud_object, fused_cloud_object_ros);
      ROS_INFO_STREAM("FUSED BOWL_OBJECT:\n" << fused_cloud_object_ros);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromPCLPointCloud2(fused_cloud_object,
                              *point_cloud_ptr);  //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>

      pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
      viewer.showCloud(point_cloud_ptr);
      while (!viewer.wasStopped())
      {
      }
    }
    if (state == State::CHECK_GRASPQUALITY)
    {
      ROS_INFO("CURRENT STATE : CHECK_GRASPQUALITY");
      bool found;
      servoing_franka::graspquality_service graspquality_service_srv;
      pcl_conversions::fromPCL(fused_cloud_object, graspquality_service_srv.request.in_cloud);
      float normals_angle;
      float normal_distvector_angle;
      if (graspquality_service_client.call(graspquality_service_srv))
      {
        bool done = false;
        while (!done)
        {
          done = graspquality_service_srv.response.status;
        }
        if (graspquality_service_srv.response.found)
        {
          normals_angle = graspquality_service_srv.response.normals_angle;
          normal_distvector_angle = graspquality_service_srv.response.normal_distvector_angle;

          if ((abs(normal_distvector_angle - 180) < NORMAL_ANGLE_TH) && abs(normals_angle - 180) < NORMAL_ANGLE_TH)
          {
            std::cout << "----------------FOUND AT------" << normal_distvector_angle << "and" << normals_angle
                      << std::endl;
            ROS_INFO_STREAM("--------------FOUND AT ------------" << normal_distvector_angle << "and " << normals_angle
                                                                  << "\n");
            found = true;
          }
          state = State::DONE;  // Change state
        }
        else
        {
          state = State::COMPUTE_PCA;  // Change state
          ROS_WARN("Grasp region - NOT FOUND");
        }
        ROS_INFO("GRASP QUALITY CALCULATION DONE");
      }
      else
      {
        ROS_ERROR("Failed to call graspquality_service");
        return 1;
      }
    }

    if (state == State::COMPUTE_PCA)
    {
      ROS_INFO("CURRENT STATE : COMPUTE PCA");
      servoing_franka::computepca_service computepca_service_srv;

      pcl_conversions::fromPCL(fused_cloud_object, computepca_service_srv.request.in_cloud);
      if (computepca_service_client.call(computepca_service_srv))
      {
        bool done = false;
        while (!done)
        {
          done = computepca_service_srv.response.status;
        }
        ROS_INFO("PCA COMPUTE DONE");
        major_vector = computepca_service_srv.response.major_vector;
        middle_vector = computepca_service_srv.response.middle_vector;
        minor_vector = computepca_service_srv.response.minor_vector;
        object_centroid = computepca_service_srv.response.centroid;

        state = State::COMPUTE_NEXTACTION;  // Change state
        ROS_INFO_STREAM("MAJOR VECTOR: \n" << major_vector);
        ROS_INFO_STREAM("MIDDLE VECTOR: \n" << middle_vector);
        ROS_INFO_STREAM("MINOR VECTOR: \n" << minor_vector);
        ROS_INFO_STREAM("CENTROID: \n" << object_centroid);
      }
      else
      {
        ROS_ERROR("Failed to call compute_pca");
        return 1;
      }
    }

    if (state == State::COMPUTE_NEXTACTION)
    {
      ROS_INFO("CURRENT STATE : COMPUTE_NEXTACTION");

      servoing_franka::pose_service pose_service_srv;
      pose_service_srv.request.object_point = object_centroid;
      pose_service_srv.request.prev_sphere_pose = current_pose;
      pose_service_srv.request.minor_eigen_vector = minor_vector;

      ROS_INFO("CURRENT STATE : COMPUTE_NEXTACTION");
      if (pose_service_client.call(pose_service_srv))
      {
        ROS_INFO("CURRENT STATE : COMPUTE_NEXTACTION");
        bool done = false;
        while (!done)
        {
          done = pose_service_srv.response.status;
        }
        ROS_INFO("COMPUTING NEXT ACTION DONE");
        next_pose.position = pose_service_srv.response.position;
        next_pose.orientation = pose_service_srv.response.orientation;
        ROS_INFO_STREAM("NEXT ACTION\n" << next_pose);

        state = State::GOTONEXTPOSE;  // Change state
      }
      else
      {
        ROS_ERROR("Failed to call compute next action");
        return 1;
      }
    }

    if (state == State::GOTONEXTPOSE)
    {
      ROS_INFO("CURRENT STATE : GOTONEXTPOSE");
      current_pose = getCurentCameraCoordinates();

      ROS_INFO_STREAM("CURRENT POSE is" << current_pose << "\n");

      servoing_franka::move_panda move_panda_srv;
      move_panda_srv.request.pose = next_pose;
      if (move_panda_client.call(move_panda_srv))
      {
        bool done = false;
        while (!done)
        {
          done = move_panda_srv.response.status;
        }
        ROS_INFO("Panda moved to goal");
        state = State::GRAB_POINTCLOUD;  // Change state
      }
      else
      {
        ROS_ERROR("Failed to call service move_panda");
        return 1;
      }
    }
  }
}