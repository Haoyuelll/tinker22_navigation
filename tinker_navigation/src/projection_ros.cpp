#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg) {
    printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
        printf("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pc2_projection");
    ros::NodeHandle n1;
    ros::NodeHandle n2;

    ros::Publisher pub = n1.advertise<PointCloud>("projected_pointcloud", 1000);

    PointCloud::Ptr msg(new PointCloud);
    PointCloud::Ptr cloud_projected(new PointCloud);
    sensor_msgs::PointCloud2 output;

    msg->header.frame_id = "camera_projection_frame";
    msg.width = 640;
    msg.height = 480;
    msg.points.resize(msg.width * msg.height);

    // msg->points.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));

    ros::Subscriber sub =
        n2.subscribe<PointCloud>("/camera/depth/image_rect_raw", 1, callback);
    ros::spin();

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
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    std::cerr << "Cloud after projection: " << std::endl;
    for (const auto& point : *cloud_projected)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z
                  << std::endl;

    pcl::toROSMsg(cloud_projected, output);
    output.header.frame_id = "camera_projection_frame";

    ros::Rate loop_rate(10);
    while (nh.ok()) {
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}