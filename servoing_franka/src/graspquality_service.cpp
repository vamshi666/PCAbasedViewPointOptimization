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


#include <servoing_franka/graspquality_service.h>

#define GRIPPER_WIDTH 0.5
#define DOT_PRODUCT_TH 0.5

#define NORMAL_ANGLE_TH 20

// ros::Publisher pub_normals;
// pcl::visualization::PCLVisualizer::Ptr cloud_viewer;



// pcl::visualization::PCLVisualizer::Ptr normalsVis (
//     pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
// {
//   // --------------------------------------------------------
//   // -----Open 3D viewer and add point cloud and normals-----
//   // --------------------------------------------------------
//   pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//   viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//   viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
//   viewer->addCoordinateSystem (1.0);
//   viewer->initCameraParameters ();
//   return (viewer);
// }

float getEuclideanDist(pcl::PointXYZRGB a, pcl::PointXYZRGB b){
  float dist = sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z)); 
  return dist;
}

float getDotProduct(pcl::Normal normal1, pcl::Normal normal2){
  Eigen::Vector3f p = normal1.getNormalVector3fMap();
  Eigen::Vector3f q = normal2.getNormalVector3fMap();
  return p.dot (q); 
}


bool getGraspQuality(servoing_franka::graspquality_service::Request &req, servoing_franka::graspquality_service::Response &res ){

  float normals_angle;
  float normal_distvector_angle;
  float sum,max_sum=0.0f;

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(req.in_cloud,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*point_cloud_ptr);


     // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (point_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.03);// 0.0075);
  ne.compute (*cloud_normals1);
  std::cout<< "Normal cloud size "<<cloud_normals1->points.size()<<std::endl;
  
  /////////////Calculate grasp quality ///////////////////////////////
  float euclidean_distance;
  bool found = false;
  for (std::size_t i = 0; i < point_cloud_ptr->points.size (); ++i){
    for(std::size_t j=0; j< cloud_normals1->points.size() && j!=i;++j){
      euclidean_distance = getEuclideanDist(point_cloud_ptr->points[i],point_cloud_ptr->points[j]);
      if(euclidean_distance>GRIPPER_WIDTH) continue;
      else{
        pcl::Normal normal1 = cloud_normals1->points[i]; //current point's normal
        pcl::Normal normal2 = cloud_normals1->points[j]; //considered point's normal

        Eigen::Vector3f distance_vector = point_cloud_ptr->points[i].getVector3fMap() - point_cloud_ptr->points[j].getVector3fMap() ;

        Eigen::Vector3f normal1_eigen = normal1.getNormalVector3fMap().normalized();
        Eigen::Vector3f normal2_eigen = normal2.getNormalVector3fMap().normalized();

        // std::cout<<normal1_eigen<<std::endl;
        // std::cout<<normal2_eigen<<std::endl;

        // float dot_product_normals = getDotProduct(normal1, normal2);
        float dot_product_normals = normal1_eigen.dot(normal2_eigen);
        dot_product_normals = dot_product_normals > 1.0 ? 1.0 : (dot_product_normals < -1.0 ? -1.0 : dot_product_normals);
        normals_angle = acos(dot_product_normals)* 180. / M_PI;;

        float dot_product_normal_distvector = distance_vector.normalized().dot(normal1_eigen);
        dot_product_normal_distvector = dot_product_normal_distvector > 1.0 ? 1.0 : (dot_product_normal_distvector < -1.0 ? -1.0 : dot_product_normal_distvector);
        normal_distvector_angle = acos(dot_product_normal_distvector)* 180. / M_PI; ;


        // std::cout<< dot_product_normals <<"  "<<dot_product_normal_distvector<<std::endl;  
        std::cout<< normals_angle<<" "<<normal_distvector_angle <<std::endl;  

        // if(dot_product_normal_distvector>1 || dot_product_normals >1){
        //   std::cout<<"Greater than 1"<<std::endl;
        // }
        // if(abs(dot_product_normals) > DOT_PRODUCT_TH && abs(dot_product_normal_distvector) > DOT_PRODUCT_TH){
        //     std::cout<<"----------------FOUND AT------" << normal1 <<"and" <<normal2 <<std::endl;
        // }
        // if((abs(normal_distvector_angle - 180) < NORMAL_ANGLE_TH) && abs(normals_angle - 180) < NORMAL_ANGLE_TH){
        // if((((int(normal_distvector_angle) % 180) < NORMAL_ANGLE_TH) || ((int(normal_distvector_angle)%180) > (180-NORMAL_ANGLE_TH))) && (((int(normals_angle) % 180) < NORMAL_ANGLE_TH) || ((int(normals_angle)%180) < (180-NORMAL_ANGLE_TH)))){
            // std::cout<<"----------------FOUND AT------" << normal1_eigen <<"and" <<normal2_eigen <<std::endl;
            // found = true;
        // }

        sum = normal_distvector_angle + normals_angle;
        if(sum > max_sum) max_sum=sum;
        if((max_sum)>300) {
            std::cout<<"----------------FOUND AT------" << normal1_eigen <<"and" <<normal2_eigen <<std::endl;
            found = true;
        }
      }
      if(found) break;
    }
    if(found) break;
  }
  if(found){
    res.normals_angle = normals_angle;
    res.normal_distvector_angle = normal_distvector_angle;
    res.found = true;
  }
  else{
    res.found = false;
  }
  res.status = true;
  ROS_INFO_STREAM("max sum is"<<max_sum);
  return true;

  //  // No need to downsample as the segmented object is already downsampled


  //  // // Perform voxel grid downsampling filtering
  //  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  //  // sor.setInputCloud (cloudPtr);
  //  // sor.setLeafSize (0.02, 0.02, 0.02);
  //  // sor.filter (*cloudFilteredPtr);

  //  // Create the normal estimation class, and pass the input dataset to it
  // pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  // ne.setInputCloud (cloud);

  // // Create an empty kdtree representation, and pass it to the normal estimation object.
  // // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  // ne.setSearchMethod (tree);

  // // Output datasets
  // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // // Use all neighbors in a sphere of radius 3cm
  // ne.setRadiusSearch (0.03);

  // // Compute the features
  // ne.compute (*cloud_normals);

  // cloud_viewer = normalsVis(cloud, cloud_normals);


  //  // pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  //  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); 

  //  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  //  //  pcl::fromPCLPointCloud2(*cloud_normals, *xyzCloudPtr);


  //  // Uncomment this to publish pointcloud with normals
  //  // sensor_msgs::PointCloud2 output_object;
  //  // pcl::toROSMsg(*cloud_normals, output_object);

  //  // pub_normals.publish(output_object);

   //--------------------
  // -----Main loop-----
  //--------------------
  // while (!cloud_viewer->wasStopped ())
  // {
  //    cloud_viewer->spinOnce (100);
  // }

}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "graspquality_service");
    ros::NodeHandle nh;
    // ros::AsyncSpinner spinner2(100); spinner2.start();
    ros::ServiceServer service = nh.advertiseService("graspquality_service", getGraspQuality);

    // Spin
    ros::spin ();
    return 0;
}