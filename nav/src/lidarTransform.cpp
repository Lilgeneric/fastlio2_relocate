#include"livox_ros_driver2/CustomMsg.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include<string>

class lidarTransformer
{
public:
    lidarTransformer():tfBuffer_(), tfListener_(tfBuffer_) 
    {
        nh_ = ros::NodeHandle("~");
        nh_.param<std::string>("laser_input", laser_input_, "pointcloud2");
        nh_.param<std::string>("laser_output", laser_output_, "lidar");
        nh_.param<std::string>("target_frame",target_frame_,"base_footprint");
        nh_.param<std::string>("source_frame",source_frame_,"laser_front_link");
        sub_ = nh_.subscribe(laser_input_, 100, &lidarTransformer::cb_transform, this);
        pub_ = nh_.advertise<livox_ros_driver2::CustomMsg>(laser_output_,100);
        ROS_INFO_STREAM("get source frame is "<<source_frame_ <<"target frame is "<<target_frame_);
    }

    void cb_transform(const livox_ros_driver2::CustomMsg &cloud_in)
    {
        // ROS_INFO("get lidar data");
        geometry_msgs::TransformStamped t_in_ = tfBuffer_.lookupTransform(target_frame_, source_frame_, ros::Time(0));

        livox_ros_driver2::CustomMsg cloud_out;
        this->doTransform(cloud_in,cloud_out,t_in_);

        pub_.publish(cloud_out);
    }
    
    void doTransform(const livox_ros_driver2::CustomMsg &cloud_in, livox_ros_driver2::CustomMsg &cloud_out,const geometry_msgs::TransformStamped& t_in)
    {
        cloud_out = cloud_in;
        // cloud_out.lidar_id = cloud_in.lidar_id;
        // cloud_out.point_num = 
        std::vector<livox_ros_driver2::CustomPoint> point_in = cloud_in.points;
        livox_ros_driver2::CustomPoint point_out;
        Eigen::Transform<float,3,Eigen::Isometry> t = Eigen::Translation3f(t_in.transform.translation.x,t_in.transform.translation.y,t_in.transform.translation.z)*
                                                    Eigen::Quaternion<float>(t_in.transform.rotation.w,t_in.transform.rotation.x,t_in.transform.rotation.y,t_in.transform.rotation.z);
        cloud_out.points.clear();
        Eigen::Vector3f point_tmp;
        for(auto p:point_in)
        {
            point_tmp = t*Eigen::Vector3f(p.x,p.y,p.z);
            point_out.x = point_tmp.x();
            point_out.y = point_tmp.y();
            point_out.z = point_tmp.z();
            point_out.reflectivity = p.reflectivity;
            point_out.line = p.line;
            point_out.offset_time = p.offset_time;
            cloud_out.points.emplace_back(point_out);
        }
    }
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    std::string laser_input_;
    std::string laser_output_;
    std::string target_frame_;
    std::string source_frame_;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_node");
    lidarTransformer node;
    ros::spin();  // 保持节点运行
    return 0;
}