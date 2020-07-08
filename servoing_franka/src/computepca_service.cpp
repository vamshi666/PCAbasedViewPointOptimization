#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <visualization_msgs/Marker.h>

#include <servoing_franka/computepca_service.h>

bool computePCA(servoing_franka::computepca_service::Request &req, servoing_franka::computepca_service::Response &res)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(req.in_cloud, pcl_pc2);

  // // Convert to PCL data type

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud_ptr);

  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cloud_ptr);
  feature_extractor.compute();

  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter(mass_center);

  pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
  pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1),
					   major_vector(2) + mass_center(2));
  pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1),
					   middle_vector(2) + mass_center(2));
  pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1),
					   minor_vector(2) + mass_center(2));

  ROS_INFO("minor vector x: %f", minor_vector(0));
  ROS_INFO("minor vector y: %f", minor_vector(1));
  ROS_INFO("minor vector z: %f", minor_vector(2));

  geometry_msgs::Vector3 majorvector_ros, middlevector_ros, minorvector_ros;
  tf::vectorEigenToMsg(major_vector.normalized().cast<double>(), majorvector_ros);
  tf::vectorEigenToMsg(middle_vector.normalized().cast<double>(), middlevector_ros);
  tf::vectorEigenToMsg(minor_vector.normalized().cast<double>(), minorvector_ros);

  res.major_vector = majorvector_ros;
  res.middle_vector = middlevector_ros;
  res.minor_vector = minorvector_ros;
  res.centroid.x = mass_center(0);
  res.centroid.y = mass_center(1);
  res.centroid.z = mass_center(2);

  res.status = true;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "computepca_service_node");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("computepca_service", computePCA);

  // Spin
  ros::spin();
  return 0;
}