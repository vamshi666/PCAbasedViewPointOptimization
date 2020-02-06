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

#include <servoing_franka/fusecloud_service.h>


class PointCloudFusion {
  protected:
        // This is primarily to save on typing(!)
        typedef pcl::PointCloud<pcl::PointXYZRGB> point_cloud_t_;

        // The fused point cloud itself
        point_cloud_t_            fused_cloud_;

        // Listener for tf frames
        tf::TransformListener     tf_listener_;

        // The name of the base frame
        std::string               base_frame_id_;

        std::string               camera_frame_id_;

  public:
        PointCloudFusion(const std::string& base_frame_id, const std::string& camera_frame_id) {
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

        bool add_cloud(servoing_franka::fusecloud_service::Request &req, servoing_franka::fusecloud_service::Response &res)
        {

            // temporary PointCloud2 intermediary
            pcl::PCLPointCloud2 tmp_pc;

            // transform the point cloud into base_frame_id
            sensor_msgs::PointCloud2 trans_ros_pc;
            tf::TransformListener listener;
            listener.waitForTransform("camera_optical_link", "world", ros::Time(0), ros::Duration(5.0), ros::Duration(2));
            if(!pcl_ros::transformPointCloud(base_frame_id_, req.current_cloud, trans_ros_pc, tf_listener_)) {
                // Failed to transform
                ROS_WARN("Dropping input point cloud");
                return false;
            }

            // Convert ROS point cloud to PCL point cloud
            // See http://wiki.ros.org/hydro/Migration for the source of this magic.
            pcl_conversions::toPCL(trans_ros_pc, tmp_pc);

            // Convert point cloud to PCL native point cloud
            point_cloud_t_ input;
            pcl::fromPCLPointCloud2(tmp_pc, input);

            // Fuse
            fused_cloud_ += input;
            pcl::toROSMsg(fused_cloud_, res.fused_cloud);
            res.status = true;
            return true;

        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusecloud_service_node");
    ros::NodeHandle nh;

    PointCloudFusion fusion("world", "camera_optical_link");

    ros::ServiceServer service = nh.advertiseService("fusecloud_service", &PointCloudFusion::add_cloud, &fusion);

    // Spin
    ros::spin ();


    return 0;
}