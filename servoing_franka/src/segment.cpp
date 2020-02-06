#include <ros/ros.h> // PCL specific includes 
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_cloud.h> 
#include <pcl/point_types.h>  
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub_tableplane;
ros::Publisher pub_object;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
 {

   // sensor_msgs::PointCloud2 trans_ros_pc;
   // listener.waitForTransform("camera_optical_link", "world", ros::Time(0), ros::Duration(5.0), ros::Duration(2));
   // if(!pcl_ros::transformPointCloud("world", cloud_msg, trans_ros_pc, listener)) {
   //       // Failed to transform
   //       ROS_WARN("Dropping input point cloud");
   //       return;
   // }

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);


    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);


    // Perform voxel grid downsampling filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.02, 0.02, 0.02);
    sor.filter (*cloudFilteredPtr);


    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

    // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);


    //perform passthrough filtering to remove table leg

    // create a pcl object to hold the passthrough filtered results
    // pcl::PointCloud<pcl::PointXYZRGB> *x_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr xCloudPtrFiltered(x_cloud_filtered);
    // pcl::PointCloud<pcl::PointXYZRGB> *xy_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyCloudPtrFiltered(xy_cloud_filtered);
    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered(xyz_cloud_filtered);


   // // Create the box filtering object
    float minX = 0.1, maxX = 1.0;
    float minY = -0.3, maxY = 0.3;
    float minZ = 0.01, maxZ = 1.0;
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ,1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ,1.0));
    boxFilter.setInputCloud(xyzCloudPtr);
    boxFilter.filter(*xyzCloudPtrFiltered);


   // Create the filtering object
   //  pcl::PassThrough<pcl::PointXYZRGB> pass1;
   //  pass1.setInputCloud (xyzCloudPtr);
   //  pass1.setFilterFieldName ("x");
   //  pass1.setFilterLimits (minX, maxX);
   //  //pass.setFilterLimitsNegative (true);
   //  pass1.filter (*xCloudPtrFiltered);

   //  Create the filtering object
   //  pcl::PassThrough<pcl::PointXYZRGB> pass2;
   //  pass2.setInputCloud (xCloudPtrFiltered);
   //  pass2.setFilterFieldName ("y");
   //  pass2.setFilterLimits (minY, maxY);
   //  //pass.setFilterLimitsNegative (true);
   //  pass2.filter (*xyCloudPtrFiltered);


   //  Create the filtering object
   //  pcl::PassThrough<pcl::PointXYZRGB> pass;
   //  pass.setInputCloud (xyzCloudPtr);
   //  pass.setFilterFieldName ("z");
   //  pass.setFilterLimits (minZ, maxZ);
   //  //pass.setFilterLimitsNegative (true);
   //  pass.filter (*xyzCloudPtrFiltered);

    // create a pcl object to hold the ransac filtered results
    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);


    // perform ransac planar filtration to remove table top
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
    // Optional
    seg1.setOptimizeCoefficients (true);
    // Mandatory
    seg1.setModelType (pcl::SACMODEL_PLANE);
    seg1.setMethodType (pcl::SAC_RANSAC);
    seg1.setDistanceThreshold (0.005);

    seg1.setInputCloud (xyzCloudPtrFiltered);
    seg1.segment (*inliers, *coefficients);


    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    //extract.setInputCloud (xyzCloudPtrFiltered);
    extract.setInputCloud (xyzCloudPtrFiltered);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*xyzCloudPtrRansacFiltered);



    sensor_msgs::PointCloud2 output_object, output_table;
    pcl::toROSMsg(*xyzCloudPtrRansacFiltered, output_object);
    pcl::toROSMsg(*xyzCloudPtrFiltered, output_table);

    pub_object.publish(output_object);
    pub_tableplane.publish(output_table);
 }

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "partial_pointcloud");
  ros::NodeHandle nh;
  tf::TransformListener listener;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_object = nh.advertise<sensor_msgs::PointCloud2> ("cloud_object", 5);
  pub_tableplane = nh.advertise<sensor_msgs::PointCloud2> ("cloud_table", 5);

   // Spin
   ros::spin ();
}