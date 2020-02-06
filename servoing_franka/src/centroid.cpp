#include <ros/ros.h> // PCL specific includes 
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_cloud.h> 
#include <pcl/point_types.h>  
#include <pcl/filters/voxel_grid.h>  
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>


#include <visualization_msgs/Marker.h>

ros::Publisher pub;
ros::Publisher vis_pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
 {
   // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*cloud_msg, cloud);

  pcl::PointXYZ min_pcl;
  pcl::PointXYZ max_pcl;
  geometry_msgs::Point centroid_pcl,centroid;

  // CentroidPoint<pcl::PointXYZ> centroid;
  // centroid.add (min_pcl.x, min_pcl.y, min_pcl.z);
  // centroid.add(max_pcl.x,max_pcl.y,max_pcl.z);

  pcl::getMinMax3D<pcl::PointXYZ>(cloud, min_pcl, max_pcl);
  centroid_pcl.x = (min_pcl.x + max_pcl.x)/2 ;
  centroid_pcl.y = (min_pcl.y + max_pcl.y)/2 ;
  centroid_pcl.z = (min_pcl.z + max_pcl.z)/2 ;

  // ROS_INFO("centroid x: %f", centroid_pcl.x);
  // ROS_INFO("centroid y: %f", centroid_pcl.y);
  // ROS_INFO("centroid z: %f", centroid_pcl.z);
   // Publish the date
   pub.publish (centroid_pcl);

   centroid.x = centroid_pcl.x;
   centroid.y = centroid_pcl.y;
   centroid.z = centroid_pcl.z;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/camera_optical_link";
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.points.push_back(centroid);

  ROS_INFO("centroid x: %f", centroid.x);
  ROS_INFO("centroid y: %f", centroid.y);
  ROS_INFO("centroid z: %f", centroid.z);

  vis_pub.publish(marker);


 }

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "centroid");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud_object", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<geometry_msgs::Point> ("centroid", 100);

  vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  
   // Spin
   ros::spin ();
}