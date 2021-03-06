#include <ros/ros.h>  // PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <servoing_franka/graspquality_service.h>

#define GRIPPER_WIDTH 0.5
#define DOT_PRODUCT_TH 0.5

#define NORMAL_ANGLE_TH 20

float getEuclideanDist(pcl::PointXYZRGB a, pcl::PointXYZRGB b)
{
  float dist = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
  return dist;
}

float getDotProduct(pcl::Normal normal1, pcl::Normal normal2)
{
  Eigen::Vector3f p = normal1.getNormalVector3fMap();
  Eigen::Vector3f q = normal2.getNormalVector3fMap();
  return p.dot(q);
}

bool getGraspQuality(servoing_franka::graspquality_service::Request &req,
                     servoing_franka::graspquality_service::Response &res)
{
  float normals_angle;
  float normal_distvector_angle;
  float sum, max_sum = 0.0f;

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(req.in_cloud, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2, *point_cloud_ptr);

  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(point_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.03);  // 0.0075);
  ne.compute(*cloud_normals1);
  std::cout << "Normal cloud size " << cloud_normals1->points.size() << std::endl;

  /////////////Calculate grasp quality ///////////////////////////////
  float euclidean_distance;
  bool found = false;
  for (std::size_t i = 0; i < point_cloud_ptr->points.size(); ++i)
  {
    for (std::size_t j = 0; j < cloud_normals1->points.size() && j != i; ++j)
    {
      euclidean_distance = getEuclideanDist(point_cloud_ptr->points[i], point_cloud_ptr->points[j]);
      if (euclidean_distance > GRIPPER_WIDTH)
        continue;
      else
      {
        pcl::Normal normal1 = cloud_normals1->points[i];  // current point's normal
        pcl::Normal normal2 = cloud_normals1->points[j];  // considered point's normal

        Eigen::Vector3f distance_vector =
            point_cloud_ptr->points[i].getVector3fMap() - point_cloud_ptr->points[j].getVector3fMap();

        Eigen::Vector3f normal1_eigen = normal1.getNormalVector3fMap().normalized();
        Eigen::Vector3f normal2_eigen = normal2.getNormalVector3fMap().normalized();

        float dot_product_normals = normal1_eigen.dot(normal2_eigen);
        dot_product_normals =
            dot_product_normals > 1.0 ? 1.0 : (dot_product_normals < -1.0 ? -1.0 : dot_product_normals);
        normals_angle = acos(dot_product_normals) * 180. / M_PI;
        ;

        float dot_product_normal_distvector = distance_vector.normalized().dot(normal1_eigen);
        dot_product_normal_distvector =
            dot_product_normal_distvector > 1.0 ?
                1.0 :
                (dot_product_normal_distvector < -1.0 ? -1.0 : dot_product_normal_distvector);
        normal_distvector_angle = acos(dot_product_normal_distvector) * 180. / M_PI;
        ;

        std::cout << normals_angle << " " << normal_distvector_angle << std::endl;

        sum = normal_distvector_angle + normals_angle;
        if (sum > max_sum)
          max_sum = sum;
        if ((max_sum) > 300)
        {
          std::cout << "----------------FOUND AT------" << normal1_eigen << "and" << normal2_eigen << std::endl;
          found = true;
        }
      }
      if (found)
        break;
    }
    if (found)
      break;
  }
  if (found)
  {
    res.normals_angle = normals_angle;
    res.normal_distvector_angle = normal_distvector_angle;
    res.found = true;
  }
  else
  {
    res.found = false;
  }
  res.status = true;
  ROS_INFO_STREAM("max sum is" << max_sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graspquality_service");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("graspquality_service", getGraspQuality);

  // Spin
  ros::spin();
  return 0;
}