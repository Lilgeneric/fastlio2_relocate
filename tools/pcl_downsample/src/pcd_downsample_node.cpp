#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <chrono>

int main(int argc, char**argv)
{
    // 1. 初始化ROS节点
    ros::init(argc, argv, "pcd_downsample_node");
    ros::NodeHandle nh("~");  // 私有命名空间，避免参数冲突

    // 2. 从launch文件或参数服务器读取参数
    std::string input_pcd_path;   // 输入PCD文件路径
    std::string output_pcd_path;  // 输出PCD文件路径
    float leaf_size_x, leaf_size_y, leaf_size_z;  // 体素大小（x,y,z）

    // 读取参数（参数名与launch文件对应，第二个参数为默认值）
    nh.param<std::string>("input_pcd", input_pcd_path, "input.pcd");
    nh.param<std::string>("output_pcd", output_pcd_path, "output_downsampled.pcd");
    nh.param<float>("leaf_size_x", leaf_size_x, 0.02f);
    nh.param<float>("leaf_size_y", leaf_size_y, 0.02f);
    nh.param<float>("leaf_size_z", leaf_size_z, 0.02f);

    // 3. 读取输入点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PCDReader reader;
    if (reader.read(input_pcd_path, *cloud) == -1) {
        ROS_ERROR("Failed to read input file: %s", input_pcd_path.c_str());
        return -1;
    }
    ROS_INFO("Successfully read point cloud: %s (number of points: %d)", input_pcd_path.c_str(), (int)cloud->size());

    // 4. 下采样处理（复用核心逻辑）
    auto start_time = std::chrono::steady_clock::now();
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);  // 使用launch配置的体素大小
    sor.filter(*cloud_filtered);
    auto end_time = std::chrono::steady_clock::now();

    // 5. 保存输出点云
    pcl::PCDWriter writer;
    writer.write(output_pcd_path, *cloud_filtered);
    ROS_INFO("Downsampling completed, saved to: %s (number of points: %d, time consumed: %d ms)",
             output_pcd_path.c_str(),
             (int)cloud_filtered->size(),
             (int)std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());

    return 0;
}