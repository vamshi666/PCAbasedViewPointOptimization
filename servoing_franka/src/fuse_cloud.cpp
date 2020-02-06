#include <string.h>

#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

ros::Publisher fused_cloud_publisher;

class PointCloudFusion {
  protected:
    // This is primarily to save on typing(!)
    typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t_;

    // The fused point cloud itself
    point_cloud_t_            fused_cloud_;

    // Listener for tf frames
    tf::TransformListener     tf_listener_;

    // The name of the base frame
    std::string               base_frame_id_;

    std::string               camera_frame_id_;

    ros::Publisher            pub_;

    // publish the fused cloud
    void publish_() const {
      // temporary PointCloud2 intermediary
      pcl::PCLPointCloud2 tmp_pc;

      // Convert fused from PCL native type to ROS
      pcl::toPCLPointCloud2(fused_cloud_, tmp_pc);

      sensor_msgs::PointCloud2 ros_pc2,published_pc;
      pcl_conversions::fromPCL(tmp_pc, ros_pc2);
      ROS_INFO("publishign");

      std::cout<<camera_frame_id_<<std::endl;

      if(!pcl_ros::transformPointCloud(camera_frame_id_, ros_pc2, published_pc, tf_listener_)) {
        // Failed to transform
        ROS_WARN("Dropping input point cloud. Something went wrong during publishing");
        // return;
      }

      published_pc.header.frame_id = camera_frame_id_;

      // Publish the data
      pub_.publish(published_pc);
    }

  public:
    PointCloudFusion(const std::string& base_frame_id, const std::string& camera_frame_id, const ros::Publisher& pub) : pub_(pub) {
      set_base_frame_id(base_frame_id);
      set_camera_frame_id(camera_frame_id);
    }
    ~PointCloudFusion() { }

    // get the base frame id
    const std::string base_frame_id() const { return base_frame_id_; }

    // update base frame id - this will reset the fused point cloud
    void set_base_frame_id(const std::string& base_frame_id) {
      // clear current fused point cloud on a base frame change
      fused_cloud_.clear();

      // record new frame
      base_frame_id_ = base_frame_id;
    }

    // update base frame id - this will reset the fused point cloud
    void set_camera_frame_id(const std::string& camera_frame_id) {
      // clear current fused point cloud on a base frame change
      fused_cloud_.clear();

      // record new frame
      camera_frame_id_ = camera_frame_id;
    }

    // callback when a new point cloud is available
    void add_cloud(const sensor_msgs::PointCloud2& ros_pc)
    {
      // temporary PointCloud2 intermediary
      pcl::PCLPointCloud2 tmp_pc;

      // transform the point cloud into base_frame_id
      sensor_msgs::PointCloud2 trans_ros_pc;
      if(!pcl_ros::transformPointCloud(base_frame_id_, ros_pc, trans_ros_pc, tf_listener_)) {
        // Failed to transform
        ROS_WARN("Dropping input point cloud");
        return;
      }

      // Convert ROS point cloud to PCL point cloud
      // See http://wiki.ros.org/hydro/Migration for the source of this magic.
      pcl_conversions::toPCL(trans_ros_pc, tmp_pc);

      // Convert point cloud to PCL native point cloud
      point_cloud_t_ input;
      pcl::fromPCLPointCloud2(tmp_pc, input);

      // Fuse
      fused_cloud_ += input;

      // Publish fused cloud
      publish_();
    }
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "fuse_cloud_node");
  ros::NodeHandle nh;

   PointCloudFusion fusion("/world", "/camera_optical_link", nh.advertise<sensor_msgs::PointCloud2>("/fused_points", 11));
  // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/cloud_object", 1, &PointCloudFusion::add_cloud, &fusion);
   // Spin
   ros::spin ();
}