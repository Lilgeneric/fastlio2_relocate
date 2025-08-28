#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>

class InitialPoseToTF {
public:
    InitialPoseToTF() : has_received_pose_(false) {
        // 初始化静态TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

        // 订阅/initialpose话题
        sub_ = nh_.subscribe("/initialpose", 10, &InitialPoseToTF::initialPoseCallback, this);

        // 启动定时器持续发布TF（10Hz）
        timer_ = nh_.createTimer(ros::Duration(0.1), &InitialPoseToTF::timerCallback, this);

        ROS_INFO("InitialPoseToTF has started, waiting for initial pose...");
    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        
        // 更新变换数据
        transformStamped_.header.stamp = ros::Time::now();
        transformStamped_.header.frame_id = "map";
        transformStamped_.child_frame_id = "odom";
        transformStamped_.transform.translation.x = msg->pose.pose.position.x;
        transformStamped_.transform.translation.y = msg->pose.pose.position.y;
        transformStamped_.transform.translation.z = msg->pose.pose.position.z;
        transformStamped_.transform.rotation = msg->pose.pose.orientation;

        has_received_pose_ = true;
        ROS_INFO("Updated TF transform and will publish continuously:\n"
                 "Translation: (%.2f, %.2f, %.2f)\n"
                 "Rotation: (%.2f, %.2f, %.2f, %.2f)",
                 transformStamped_.transform.translation.x,
                 transformStamped_.transform.translation.y,
                 transformStamped_.transform.translation.z,
                 transformStamped_.transform.rotation.x,
                 transformStamped_.transform.rotation.y,
                 transformStamped_.transform.rotation.z,
                 transformStamped_.transform.rotation.w);
    }

    void timerCallback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        if (has_received_pose_) {
            // 持续发布最新的静态TF变换
            tf_broadcaster_->sendTransform(transformStamped_);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Timer timer_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    geometry_msgs::TransformStamped transformStamped_;
    std::mutex transform_mutex_;
    bool has_received_pose_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "initial_pose_to_tf");
    InitialPoseToTF node;
    ros::spin();
    return 0;
}