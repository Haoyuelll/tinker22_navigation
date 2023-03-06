#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_create");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub =
        nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(
        new pcl::PointCloud<pcl::PointXYZ>);
        
    sensor_msgs::PointCloud2 output;

    // Fill in the cloud data
    cloud->width = 640;
    cloud->height = 480;
    cloud->points.resize(cloud->width * cloud->height);

    for (auto& point : *cloud) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before projection: " << std::endl;
    for (const auto& point : *cloud)
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
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    std::cerr << "Cloud after projection: " << std::endl;
    for (const auto& point : *cloud_projected)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z
                  << std::endl;

    pcl::toROSMsg(cloud_projected, output);
    output.header.frame_id = "camera_pointcloud_projected";

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return (0);
}
