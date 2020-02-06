#include <ros/ros.h> // PCL specific includes 
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>


#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_cloud.h> 
#include <pcl/point_types.h>  
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>


ros::Publisher pub_normals;
pcl::visualization::PCLVisualizer::Ptr cloud_viewer;


pcl::visualization::PCLVisualizer::Ptr normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
 {

   pcl::PCLPointCloud2 pcl_pc2;
   pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::fromPCLPointCloud2(pcl_pc2,*cloud);


   // No need to downsample as the segmented object is already downsampled


   // // Perform voxel grid downsampling filtering
   // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
   // sor.setInputCloud (cloudPtr);
   // sor.setLeafSize (0.02, 0.02, 0.02);
   // sor.filter (*cloudFilteredPtr);

   // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  cloud_viewer = normalsVis(cloud, cloud_normals);

  ROS_INFO("NORMAL POINTS:\n");
   for (std::size_t i = 0; i < cloud_normals->points.size(); ++i)
    std::cout << "    " << cloud_normals->points[i]<<"            "
                          << cloud_normals->points[i].normal_x <<","
                         << cloud_normals->points[i].normal_y<<","
                         << cloud_normals->points[i].normal_z<< std::endl;


   // pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); 

   // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
   //  pcl::fromPCLPointCloud2(*cloud_normals, *xyzCloudPtr);


   // Uncomment this to publish pointcloud with normals
   // sensor_msgs::PointCloud2 output_object;
   // pcl::toROSMsg(*cloud_normals, output_object);

   // pub_normals.publish(output_object);

   //--------------------
  // -----Main loop-----
  //--------------------
  cloud_viewer->spinOnce (100);

 }




int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "graspquality_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/cloud_object", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_normals = nh.advertise<sensor_msgs::PointCloud2> ("cloud_normals", 5);

   // Spin
   ros::spin ();
}