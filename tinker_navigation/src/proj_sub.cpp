#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <boost/foreach.hpp>

#include "proj.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr msg(new PointCloud);

void callback(const PointCloud::ConstPtr& msg) {
    printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
        printf("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "proj_sub");
    ros::NodeHandle n2;
    ros::Subscriber sub =
        n2.subscribe<PointCloud>("/camera/depth/pointcloud", 1, callback);
    ros::spin();
}
