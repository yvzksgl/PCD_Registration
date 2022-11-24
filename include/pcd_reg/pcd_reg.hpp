#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std::chrono_literals;

class PcdRegNode : public rclcpp::Node{
public:
    PcdRegNode() : Node("PcdRegistrator"){
        this->declare_parameter("input_pcd1_path","pcds/capture0001.pcd");
        this->declare_parameter("input_pcd2_path","pcds/capture0002.pcd");
        this->declare_parameter("out_pcd_path","pcds/transformed.pcd");
        this->declare_parameter("algorithm","gicp");
        timer_ = this->create_wall_timer(500ms, std::bind(&PcdRegNode::pcd_pub, this));
        this->in_pub1=this->create_publisher<sensor_msgs::msg::PointCloud2>("capture1",10);
        this->in_pub2=this->create_publisher<sensor_msgs::msg::PointCloud2>("capture2",10);
        this->out_pub=this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed",10);
        PcdRegNode::pcd_reader();
    }
private:
    int pcd_reader();
    void pcd_preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void ndt_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr,
                          Eigen::Matrix4f);
    void gicp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void pcd_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void pcd_coloring(pcl::PointCloud<pcl::PointXYZ>::Ptr,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void pcd_pub();
    void pcd_viewer();
    pcl::PointCloud<pcl::PointXYZRGB> target_cloudrgb;
    pcl::PointCloud<pcl::PointXYZRGB> input_cloudrgb;
    pcl::PointCloud<pcl::PointXYZRGB> output_cloudrgb;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr in_pub1;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr in_pub2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr out_pub;
};
