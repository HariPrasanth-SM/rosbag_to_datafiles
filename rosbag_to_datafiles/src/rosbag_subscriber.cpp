#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/io/pcd_io.h"
#include "fstream"
#include <iomanip> 

class RosbagSubscriber : public rclcpp::Node {
public:
    RosbagSubscriber() : Node("rosbag_subscriber") {

        callback_group_subscriber1_ = this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_subscriber2_ = this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_subscriber2_;

        // Initialize subscriber to odometry
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 1000, std::bind(&RosbagSubscriber::odomCallback, this, std::placeholders::_1), sub1_opt);

        // Initialize subscriber to point cloud
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "velodyne_points", 1000, std::bind(&RosbagSubscriber::pointCloudCallback, this, std::placeholders::_1), sub2_opt);

        // Initialize service to write data into textfiles
        datadump_service_ = this->create_service<std_srvs::srv::Empty>(
            "datadump_service", std::bind(&RosbagSubscriber::handleServiceRequest, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "odom msg Received");

        double msg_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        odom_timestamp.push_back(msg_time);

        odom_linear_velocity.push_back(msg->twist.twist.linear.x);
        odom_angular_velocity.push_back(msg->twist.twist.angular.z);

        odom_pose_x.push_back(msg->pose.pose.position.x);
        odom_pose_y.push_back(msg->pose.pose.position.y);
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "cloudReceived");

        double msg_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        cloud_timestamp.push_back(msg_time);
        
        // Convert point cloud message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Write point cloud to a PCD file
        std::string filename = std::to_string(msg_time) + ".pcd";
        pcl::io::savePCDFileASCII(filename, *cloud);
        RCLCPP_INFO(this->get_logger(), "Saved point cloud data to %s", filename.c_str());
    }

    void handleServiceRequest(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                              std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received service request");

        // Writing the control commands
        std::string odom_commands_filename = "odom_control_commands.txt";
        std::ofstream odom_commands_file(odom_commands_filename);
        if (!odom_commands_file.is_open()) 
        {
            std::cerr << "Error: Unable to open file " << odom_commands_filename << "." << std::endl;
            exit(1);
        }
        for (std::size_t i = 0; i < odom_timestamp.size(); ++i) 
        {
            odom_commands_file << std::fixed << std::setprecision(8) << odom_timestamp[i] << ", " << odom_linear_velocity[i] << ", " << odom_angular_velocity[i] << std::endl;
        }
        odom_commands_file.close();

        // Writing the odom position
        std::string odom_pose_filename = "odom_positions.txt";
        std::ofstream odom_pose_file(odom_pose_filename);
        if (!odom_pose_file.is_open()) 
        {
            std::cerr << "Error: Unable to open file " << odom_pose_filename << "." << std::endl;
            exit(1);
        }
        for (std::size_t i = 0; i < odom_timestamp.size(); ++i) 
        {
            odom_pose_file <<  std::fixed << std::setprecision(8) << odom_timestamp[i] << ", " << odom_pose_x[i] << ", " << odom_pose_y[i] << std::endl;
        }
        odom_pose_file.close();

        // Writing the odom position
        std::string cloud_timestamps_filename = "cloud_timestamps.txt";
        std::ofstream cloud_timestamps_file(cloud_timestamps_filename);
        if (!cloud_timestamps_file.is_open()) 
        {
            std::cerr << "Error: Unable to open file " << cloud_timestamps_filename << "." << std::endl;
            exit(1);
        }
        for (std::size_t i = 0; i < cloud_timestamp.size(); ++i) 
        {
            cloud_timestamps_file <<  std::fixed << std::setprecision(8) << cloud_timestamp[i] << std::endl;
        }
        cloud_timestamps_file.close();
        
        RCLCPP_INFO(this->get_logger(), "Data dumped to files; Returning empty response");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr datadump_service_;

    std::vector<double> odom_timestamp;
    std::vector<double> odom_pose_x;
    std::vector<double> odom_pose_y;
    std::vector<double> odom_linear_velocity;
    std::vector<double> odom_angular_velocity;
    std::vector<double> cloud_timestamp;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosbagSubscriber>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
