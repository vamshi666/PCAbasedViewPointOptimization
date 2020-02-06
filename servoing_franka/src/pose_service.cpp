#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <servoing_franka/pose_service.h>

float step = 10.0f; //in degrees

ros::Publisher marker_pub;

bool getPose(servoing_franka::pose_service::Request &req, servoing_franka::pose_service::Response &res ){
    
    ROS_INFO("pose_service calculation STARTED");
    // tf::Vector3 axis,tx,ty;  // axis corresponds to tz
    float angle = M_PI*step/180;
    Eigen::Vector3d current_position(req.prev_sphere_pose.position.x, req.prev_sphere_pose.position.y, req.prev_sphere_pose.position.z);
    Eigen::Vector3d object_position(req.object_point.x, req.object_point.y, req.object_point.z);
    Eigen::Vector3d initial_vector = object_position - current_position;
    // tf::Vector3 initial_vector(req.prev_sphere_pose.position.x - req.object_point.x, req.prev_sphere_pose.position.y - req.object_point.y, req.prev_sphere_pose.position.z - req.object_point.z  );
    Eigen::Vector3d minor_eigen_vector(req.minor_eigen_vector.x,req.minor_eigen_vector.y,req.minor_eigen_vector.z);
    minor_eigen_vector = minor_eigen_vector - object_position;
    Eigen::Vector3d axis = initial_vector.normalized().cross(minor_eigen_vector.normalized());
    // axis.normalized();
    // tf::Quaternion q(axis, -1.0*acos(initial_vector.dot(next_vector)));
    // res.quaternion(axis, -1.0*acos(initial_vector.dot(next_vector)));

    // res.quaternion.x = axis.x()*sin(angle/2);
    // res.quaternion.y = axis.y()*sin(angle/2);
    // res.quaternion.z = axis.z()*sin(angle/2);
    // res.quaternion.w = cos(angle/2);

    Eigen::Quaterniond quaternion= Eigen::Quaterniond{Eigen::AngleAxisd{angle, Eigen::Vector3d{axis.normalized()}}};
    Eigen::Matrix3d R = quaternion.toRotationMatrix();
    Eigen::Vector3d resultant_position = R*current_position.normalized();
    Eigen::Quaterniond quaternion_eigen, resultant_quaternion;
    // tf::quaternionMsgToEigen(req.prev_sphere_pose.orientation, quaternion_eigen);           // Uncomment these to just transform original orientation
    // resultant_quaternion = quaternion*quaternion_eigen;

    Eigen::Vector3d final_vector(object_position - resultant_position);
    Eigen::Vector3d rz = final_vector.normalized();
    Eigen::Vector3d ybasis(0,1,0);
    Eigen::Vector3d rx = rz.cross(ybasis).normalized();
    Eigen::Vector3d ry = rz.cross(rx).normalized();
    Eigen::Matrix3d orientation;
    orientation << rx,ry,rz;
    resultant_quaternion = orientation;

    std::cout<<"Quaternion output::\n"<<resultant_quaternion.vec()<<std::endl;
    std::cout<<"Quaternion  w output::\n"<<resultant_quaternion.w()<<std::endl;


    // visualization_msgs::Marker arrow;
    // arrow.header.frame_id = "/world";
    // arrow.header.stamp = ros::Time::now();

    // arrow.ns = "lines";
    // arrow.id = 1;

    // arrow.type = visualization_msgs::Marker::ARROW;
    // arrow.action = visualization_msgs::Marker::ADD;

    // tf::pointEigenToMsg(resultant_position, arrow.pose.position);
    // tf::quaternionEigenToMsg(resultant_quaternion, arrow.pose.orientation);

    // // arrow.pose.orientation.x = orientation;

    // arrow.scale.x = 0.5;
    // arrow.scale.y = 0.01;
    // arrow.scale.z = 0.01;

    // arrow.color.g = 1.0f;
    // arrow.color.a = 1.0;
    // arrow.color.r = 0.0f;
    // arrow.color.b = 0.0f;

    // arrow.lifetime = ros::Duration(200);

    // marker_pub.publish(arrow);

    tf::pointEigenToMsg(0.7*resultant_position, res.position);  //0.7m viewsphere
    tf::quaternionEigenToMsg(resultant_quaternion, res.orientation);
    res.status = true;

    ROS_INFO("pose_service calculation DONE");
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_service");
    ros::NodeHandle nh;
    // ros::AsyncSpinner spinner2(100); spinner2.start();

    ros::ServiceServer service = nh.advertiseService("pose_service", getPose);
    marker_pub = nh.advertise<visualization_msgs::Marker>("lines", 10);

    // Spin
    ros::spin ();


    return 0;
}