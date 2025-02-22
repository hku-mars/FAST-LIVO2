// #include "utility.h"
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>

#include "pcl_struct.hpp"

std::string output_type;

static int RING_ID_MAP_RUBY[] = {
    3,   66, 33,  96,  11,  74,  41,  104, 19,  82,  49,  112, 27, 90,  57,  120, 35, 98,  1,
    64,  43, 106, 9,   72,  51,  114, 17,  80,  59,  122, 25,  88, 67,  34,  97,  0,  75,  42,
    105, 8,  83,  50,  113, 16,  91,  58,  121, 24,  99,  2,   65, 32,  107, 10,  73, 40,  115,
    18,  81, 48,  123, 26,  89,  56,  7,   70,  37,  100, 15,  78, 45,  108, 23,  86, 53,  116,
    31,  94, 61,  124, 39,  102, 5,   68,  47,  110, 13,  76,  55, 118, 21,  84,  63, 126, 29,
    92,  71, 38,  101, 4,   79,  46,  109, 12,  87,  54,  117, 20, 95,  62,  125, 28, 103, 6,
    69,  36, 111, 14,  77,  44,  119, 22,  85,  52,  127, 30,  93, 60};
static int RING_ID_MAP_16[] = {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8};

ros::Subscriber subRobosensePC;
ros::Publisher pubRobosensePC;

template <typename T>
bool has_nan(T point) {
    // remove nan point, or the feature assocaion will crash, the surf point will containing nan
    // points pcl remove nan not work normally ROS_ERROR("Containing nan point!");
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
        return true;
    } else {
        return false;
    }
}

template <typename T>
void publish_points(T& new_pc, const sensor_msgs::PointCloud2& old_msg) {
    // pc properties
    new_pc->is_dense = true;

    // publish
    sensor_msgs::PointCloud2 pc_new_msg;
    pcl::toROSMsg(*new_pc, pc_new_msg);
    pc_new_msg.header = old_msg.header;
    pc_new_msg.header.stamp = ros::Time(old_msg.header.stamp.toSec() - 0.1 + 0.071);
    pc_new_msg.header.frame_id = "velodyne";
    pubRobosensePC.publish(pc_new_msg);
}

void rsHandler(sensor_msgs::PointCloud2 pc_msg) {
    pcl::PointCloud<RsPointXYZIRT>::Ptr pc(new pcl::PointCloud<RsPointXYZIRT>());
    pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_new(new pcl::PointCloud<VelodynePointXYZIRT>());
    pcl::fromROSMsg(pc_msg, *pc);

    // to new pointcloud
    int count_point{0}, pt_size{28800};
    pt_size = pc->points.size();
    for (int point_id = 0; point_id < pt_size; ++point_id) {
        count_point++;

        if (has_nan(pc->points[point_id]))
            continue;
        Eigen::Vector3d pt_vec(pc->points[point_id].x, pc->points[point_id].y,
                               pc->points[point_id].z);
        // pt_vec = rta_R * pt_vec + rta_T;

        VelodynePointXYZIRT new_point;
        new_point.x = pt_vec.x();
        new_point.y = pt_vec.y();
        new_point.z = pt_vec.z();

        new_point.intensity = pc->points[point_id].intensity;

        // remap ring id
        if (pc->height == 16) {
            new_point.ring = RING_ID_MAP_16[point_id / pc->width];
        } else if (pc->height == 128) {
            new_point.ring = RING_ID_MAP_RUBY[point_id % pc->height];
        }

        // remap time  0.1s
        new_point.time = float(pc->points[point_id].timestamp - pc->points[0].timestamp + 0.1);

        // if (count_point % 1000 == 0) {
        //     std::cout << "intensity: " << new_point.intensity << "    ";
        //     std::cout << "ring: " << new_point.ring << "    ";
        //     std::cout << "time: " << new_point.time << "\n";
        // }
        pc_new->points.push_back(new_point);
    }

    // std::cout << "\n" << pt_size << "\n\n";
    publish_points(pc_new, pc_msg);
}

std::string lidar_topic, velo_topic;
int main(int argc, char** argv) {
    ros::init(argc, argv, "rs_converter");
    ros::NodeHandle nh;

    nh.param<std::string>("common/rs_topic", lidar_topic, "/rslidar_points");
    nh.param<std::string>("common/lid_topic", velo_topic, "/velodyne_points");

    subRobosensePC = nh.subscribe(lidar_topic, 100000, rsHandler);
    pubRobosensePC = nh.advertise<sensor_msgs::PointCloud2>(velo_topic, 100000);

    ROS_INFO("Listening to /rslidar_points.");
    ros::spin();
    return 0;
}
