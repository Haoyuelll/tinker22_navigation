#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/foreach.hpp>
#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class SubscribeAndPublish {
   public:
    SubscribeAndPublish() {
        // Topic you want to publish
        pub_ = n_.advertise<PointCloud>("/projected_pointcloud", 1000);

        // Topic you want to subscribe
        sub_ = n_.subscribe("/camera/depth/image_rect_raw", 1,
                            &SubscribeAndPublish::callback, this);
    }

    // void resize(width, height, std::size_t count) {
    //     points.resize(count);
    //     if (width * height != count) {
    //         width = static_cast<std::uint32_t>(count);
    //         height = 1;
    //     }
    // }

    void callback(const PointCloud& input) {
        // PointCloud output;

        // input.header.frame_id = "camera_projection_frame";
        // input.resize(640, 480);

        std::cerr << "Cloud before projection: " << std::endl;
        for (auto& point : input)
            std::cerr << "    " << point.x << " " << point.y << " " << point.z
                      << std::endl;

        // Create a set of planar coefficients with X=Y=0,Z=1
        // Create the filtering object
        PointCloud cloud_projected;
        for (auto& point : input) {
            pcl::PointXYZ p(point.x, point.y, point.z);
            if (p.z > 0.01 && p.z < 1.35) {
                p.z = 0;
                cloud_projected.push_back(p);
            }
        }

        std::cerr << "Cloud after projection: " << std::endl;
        for (auto& point : cloud_projected)
            std::cerr << "    " << point.x << " " << point.y << " " << point.z
                      << std::endl;

        // pcl::toROSMsg(cloud_projected, output);
        cloud_projected.header.frame_id = "camera_projection_frame";

        pub_.publish(cloud_projected);
    }

   private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

};  // End of class SubscribeAndPublish

int main(int argc, char** argv) {
    // Initiate ROS
    ros::init(argc, argv, "projection_sub_and_pub");

    SubscribeAndPublish proj_;

    ros::spin();

    return 0;
}