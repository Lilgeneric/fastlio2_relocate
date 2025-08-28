#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>  // 添加半径滤波器头文件
#include <string>
#include <cmath>

std::string file_directory, file_name, map_topic_name, projection_plane;
std::string pcd_file;
const std::string pcd_format = ".pcd";

double map_resolution = 0.05;
double filter_radius = 0.5;      // 默认滤波半径(m)
int min_neighbors = 5;           // 默认最小邻居数

pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);
nav_msgs::OccupancyGrid map_topic_msg;

void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, nav_msgs::OccupancyGrid& msg);

// 添加半径滤波函数
pcl::PointCloud<pcl::PointXYZ>::Ptr applyRadiusFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (filter_radius <= 0 || min_neighbors <= 0) {
        ROS_WARN("Invalid filter parameters, skipping radius filter");
        return cloud;
    }

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(filter_radius);
    outrem.setMinNeighborsInRadius(min_neighbors);
    outrem.filter(*filtered_cloud);

    ROS_INFO("Radius filter applied: %zu -> %zu points", cloud->points.size(), filtered_cloud->points.size());
    return filtered_cloud;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_filters");
    ros::NodeHandle nh, private_nh("~");

    // 添加半径滤波参数
    private_nh.param("file_directory", file_directory, std::string("/home/user/PCD/"));
    private_nh.param("file_name", file_name, std::string("map"));
    private_nh.param("map_resolution", map_resolution, 0.05);
    private_nh.param("map_topic_name", map_topic_name, std::string("map"));
    private_nh.param("projection_plane", projection_plane, std::string("xy"));
    private_nh.param("filter_radius", filter_radius, 0.5);
    private_nh.param("min_neighbors", min_neighbors, 5);

    pcd_file = file_directory + file_name + pcd_format;
    ROS_INFO("Loading PCD: %s", pcd_file.c_str());
    ROS_INFO("Filter params: radius=%.2fm, min_neighbors=%d", filter_radius, min_neighbors);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", pcd_file.c_str());
        return -1;
    }

    ROS_INFO("Loaded %zu points.", pcd_cloud->points.size());

    // 应用半径滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = applyRadiusFilter(pcd_cloud);
    
    SetMapTopicMsg(filtered_cloud, map_topic_msg);

    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1);
    ros::Rate loop_rate(1.0);

    while (ros::ok()) {
        map_pub.publish(map_topic_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, nav_msgs::OccupancyGrid& msg) {
    if (cloud->points.empty()) {
        ROS_WARN("Empty point cloud.");
        return;
    }

    double min_a = 0, max_a = 0, min_b = 0, max_b = 0;
    int axis_a = 0, axis_b = 1;

    // 选择投影平面
    if (projection_plane == "xz") {
        axis_a = 0; // x
        axis_b = 2; // z
    } else if (projection_plane == "yz") {
        axis_a = 1; // y
        axis_b = 2; // z
    } else {
        axis_a = 0; // 默认 xy
        axis_b = 1;
    }

    // 计算边界
    min_a = max_a = cloud->points[0].data[axis_a];
    min_b = max_b = cloud->points[0].data[axis_b];
    for (const auto& pt : cloud->points) {
        double a = pt.data[axis_a];
        double b = pt.data[axis_b];
        if (a < min_a) min_a = a;
        if (a > max_a) max_a = a;
        if (b < min_b) min_b = b;
        if (b > max_b) max_b = b;
    }

    int width = std::ceil((max_a - min_a) / map_resolution);
    int height = std::ceil((max_b - min_b) / map_resolution);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.info.map_load_time = ros::Time::now();
    msg.info.resolution = map_resolution;
    msg.info.width = width;
    msg.info.height = height;

    // 设置 origin
    msg.info.origin.position.x = (axis_a == 0) ? min_a : ((axis_b == 0) ? min_b : 0.0);
    msg.info.origin.position.y = (axis_a == 1) ? min_a : ((axis_b == 1) ? min_b : 0.0);
    msg.info.origin.position.z = (axis_a == 2) ? min_a : ((axis_b == 2) ? min_b : 0.0);
    msg.info.origin.orientation.w = 1.0;

    msg.data.assign(width * height, 0); // 初始全0

    // 填充地图
    for (const auto& pt : cloud->points) {
        int i = (pt.data[axis_a] - min_a) / map_resolution;
        int j = (pt.data[axis_b] - min_b) / map_resolution;
        if (i < 0 || i >= width || j < 0 || j >= height) continue;
        msg.data[i + j * width] = 100;
    }

    ROS_INFO("OccupancyGrid generated: %d x %d, projection plane: %s", 
             width, height, projection_plane.c_str());
}