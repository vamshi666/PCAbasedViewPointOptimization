#include <ros/ros.h> // PCL specific includes 
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>


#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_cloud.h> 
#include <pcl/point_types.h>  
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


#include <servoing_franka/segment_service.h>



bool getSegmentedObject(servoing_franka::segment_service::Request &req, servoing_franka::segment_service::Response &res ){

    sensor_msgs::PointCloud2 trans_ros_pc;
    tf::TransformListener listener;
   listener.waitForTransform("camera_optical_link", "world", ros::Time(0), ros::Duration(5.0), ros::Duration(2));
   if(!pcl_ros::transformPointCloud("/world", req.in_cloud, trans_ros_pc, listener)) {
         // Failed to transform
         ROS_WARN("Dropping input point cloud");
         return false;
   }
    
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);


    ROS_INFO("Processing cloud");
    // Convert to PCL data type
    pcl_conversions::toPCL(trans_ros_pc, *cloud);


    // Perform voxel grid downsampling filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.005, 0.005, 0.005);
    sor.filter (*cloudFilteredPtr);


    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

    // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);


    //perform passthrough filtering to remove table leg

    // create a pcl object to hold the passthrough filtered results
    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);


    // Create the box filtering object
    float minX = 0.3, maxX=1;
    float minY = -0.3, maxY = 0.3;
    float minZ = 0.01, maxZ = 1;
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(xyzCloudPtr);
    boxFilter.filter(*xyzCloudPtrFiltered);

    // Create the filtering object
    // pcl::PassThrough<pcl::PointXYZRGB> pass;
    // pass.setInputCloud (xyzCloudPtr);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (.01, 1);
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits(0.3,1);
    // //pass.setFilterLimitsNegative (true);
    // pass.filter (*xyzCloudPtrFiltered);

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


        ROS_INFO("Almost done");
    sensor_msgs::PointCloud2 output_object, output_table;
    pcl::toROSMsg(*xyzCloudPtrRansacFiltered, output_object);
    pcl::toROSMsg(*xyzCloudPtrRansacFiltered, res.out_cloud);

    ROS_INFO("Done");
    res.status = true;    
    // pcl::toROSMsg(*xyzCloudPtrFiltered, output_table);

    // pub_object.publish(output_object);
    // pub_tableplane.publish(output_table);
    return true;
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segment_service_node");
    ros::NodeHandle nh;

    // ros::AsyncSpinner spinner2(100); spinner2.start();

    ros::ServiceServer service = nh.advertiseService("segment_service", getSegmentedObject);

    // Spin
    ros::spin ();


    return 0;
}