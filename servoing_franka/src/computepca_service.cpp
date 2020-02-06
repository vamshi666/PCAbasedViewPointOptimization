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



// geometry_msgs :: Point toPoint(pcl::PointXYZ pcl_point){
// 	geometry_msgs::Point p;
// 	p.x = pcl_point.x;
// 	p.y=pcl_point.y;
// 	p.z = pcl_point.z;
// 	return p;
// }


// void rviz_visualize(pcl::PointXYZ center, pcl::PointXYZ x_axis, pcl::PointXYZ y_axis, pcl::PointXYZ z_axis){
// 	visualization_msgs::Marker points, line_list;
// 	geometry_msgs::Point p;
// 	line_list.header.frame_id = "/camera_optical_link";

// 	line_list.header.stamp = ros::Time::now();

// 	line_list.action = visualization_msgs::Marker::ADD;

// 	line_list.pose.orientation.w = 1.0;

// 	line_list.id = 2;

// 	line_list.type = visualization_msgs::Marker::LINE_LIST;

// 	line_list.scale.x = 0.1;

// 	// Line list is red
// 	line_list.color.r = 1.0;
// 	line_list.color.a = 1.0;
// 	// The line list needs two points for each line
	
// 	p = toPoint(center);
// 	line_list.points.push_back(p);
	
// 	p=toPoint(x_axis);
// 	line_list.points.push_back(p);

// 	// Line list is red
// 	// line_list.color.g = 1.0;
// 	// line_list.color.a = 1.0;
// 	// The line list needs two points for each line
	
// 	p = toPoint(center);
// 	line_list.points.push_back(p);
	
// 	p=toPoint(y_axis);
// 	line_list.points.push_back(p);

// 	// Line list is red
// 	// line_list.color.b = 1.0;
// 	// line_list.color.a = 1.0;
// 	// The line list needs two points for each line
	
// 	p = toPoint(center);
// 	line_list.points.push_back(p);
	
// 	p=toPoint(z_axis);
// 	line_list.points.push_back(p);
// 	pca_axes.publish(line_list);


// }

bool computePCA(servoing_franka::computepca_service::Request &req, servoing_franka::computepca_service::Response &res ){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(req.in_cloud,pcl_pc2);
    
	// // Convert to PCL data type
    
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::fromPCLPointCloud2(pcl_pc2, *cloud_ptr);

	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud (cloud_ptr);
	feature_extractor.compute ();
 
	// std::vector <float> moment_of_inertia;
	// std::vector <float> eccentricity;
 
 
	// pcl::PointXYZ min_point_AABB;
	// pcl::PointXYZ max_point_AABB;
	// pcl::PointXYZ min_point_OBB;
	// pcl::PointXYZ max_point_OBB;
	// pcl::PointXYZ position_OBB;
 
 
	// Eigen::Matrix3f rotational_matrix_OBB;
	// float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;
 
	// feature_extractor.getMomentOfInertia (moment_of_inertia);
	// feature_extractor.getEccentricity (eccentricity);
	// feature_extractor.getAABB (min_point_AABB, max_point_AABB);
	// feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	// feature_extractor.getEigenValues (major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter (mass_center);

	// major_vector = major_vector + mass_center;
	// middle_vector = middle_vector + mass_center;
	// minor_vector = minor_vector + mass_center;

	pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
	pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
	pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
	pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
 
	ROS_INFO("minor vector x: %f", minor_vector(0));
	ROS_INFO("minor vector y: %f", minor_vector(1));
	ROS_INFO("minor vector z: %f", minor_vector(2));

    geometry_msgs::Vector3 majorvector_ros, middlevector_ros, minorvector_ros;
    tf::vectorEigenToMsg(major_vector.normalized().cast<double>(), majorvector_ros);
    tf::vectorEigenToMsg(middle_vector.normalized().cast<double>(), middlevector_ros);
    tf::vectorEigenToMsg(minor_vector.normalized().cast<double>(), minorvector_ros);
    
    res.major_vector  = majorvector_ros ;   
    res.middle_vector =  middlevector_ros ;
    res.minor_vector  = minorvector_ros ;
	res.centroid.x = mass_center(0);
	res.centroid.y = mass_center(1);
	res.centroid.z = mass_center(2);

    res.status = true;
	// rviz_visualize(center, x_axis, y_axis, z_axis);

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "computepca_service_node");
    ros::NodeHandle nh;
    // ros::AsyncSpinner spinner2(100); spinner2.start();
    ros::ServiceServer service = nh.advertiseService("computepca_service", computePCA);

    // Spin
    ros::spin ();
    return 0;
}