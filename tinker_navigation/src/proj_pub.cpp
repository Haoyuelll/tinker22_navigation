#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>

#include "proj.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "proj_pub");
    ros::NodeHandle n1;
    ros::Publisher pub = n1.advertise<PointCloud>("pointcloud_projected", 1000);

    // PointCloud::Ptr msg;
    PointCloud::Ptr cloud_projected(new PointCloud);
    sensor_msgs::PointCloud2 output;

    msg->header.frame_id = "camera_depth_cloud";
    msg.width = 640;
    msg.height = 480;
    msg.points.resize(msg.width * msg.height);

    std::cerr << "Cloud before projection: " << std::endl;
    for (const auto& point : *msg)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z
                  << std::endl;

    // Create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    // coefficients->values[2] = 1.0;
    coefficients->values[2] = (values[2] > 0.01 && values[2] < 1.35) ? 1.0 : 0;
    coefficients->values[3] = 0;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(msg);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    std::cerr << "Cloud after projection: " << std::endl;
    for (const auto& point : *cloud_projected)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z
                  << std::endl;

    pcl::toROSMsg(cloud_projected, output);
    output.header.frame_id = "camera_pointcloud_projected";

    ros::Rate loop_rate(4);
    while (n1.ok()) {
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
}