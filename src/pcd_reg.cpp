#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pcd_reg.hpp"

int PcdRegNode::pcd_reader(){
    std::string input_pcd1_path;
    std::string input_pcd2_path;
    this->get_parameter("input_pcd1_path",input_pcd1_path);
    this->get_parameter("input_pcd2_path",input_pcd2_path);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd1_path,*target_cloud)==-1){
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd2_path,*input_cloud)==-1){
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    pcd_preprocess(target_cloud,input_cloud);
    return 0;
}

void PcdRegNode::pcd_preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*target_cloud,*target_cloud,index);
    pcl::removeNaNFromPointCloud(*input_cloud,*input_cloud,index);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);

    RCLCPP_INFO(this->get_logger(),"Filtered cloud contains %d  data points from target_cloud\n", filtered_cloud->size());

    pcd_registration(input_cloud,target_cloud,filtered_cloud);
}

void PcdRegNode::gicp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud){
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

/*  
    Parameter tuning doesn't provide any considerable improvement on gicp's accuracy
    Also didn't affect its speed as well.

    gicp.setMaxCorrespondenceDistance(30);
    gicp.setMaximumIterations(35);
    gicp.setTransformationEpsilon(5e-4);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setRANSACIterations(0); 
*/

    gicp.setInputSource(filtered_cloud);
    gicp.setInputTarget(input_cloud);
    gicp.align(*output_cloud);

    RCLCPP_INFO(this->get_logger(),"Generalized Iterative Closest Point has converged with score: %f",gicp.getFitnessScore());

    pcl::transformPointCloud(*target_cloud, *output_cloud, gicp.getFinalTransformation());
}

void PcdRegNode::ndt_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
                                  Eigen::Matrix4f init_guess){
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

/*  
    These parameters makes ndt approximately 3 times more accurate but,
    also makes it 30 times slower!!! 
    Nevertheless still cannot catch gicp's accuracy.  

    ndt.setMaxCorrespondenceDistance (30);
    ndt.setMaximumIterations(35);
    ndt.setTransformationEpsilon(5e-4);
    ndt.setEuclideanFitnessEpsilon (1e-6);
    ndt.setRANSACIterations(0); 
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
*/

    ndt.setInputSource(filtered_cloud);
    ndt.setInputTarget(input_cloud);
    ndt.align(*output_cloud, init_guess);

    RCLCPP_INFO(this->get_logger(),"Normal Distributions Transform has converged with score: %f",ndt.getFitnessScore());

    pcl::transformPointCloud(*target_cloud, *output_cloud, ndt.getFinalTransformation());
}

void PcdRegNode::pcd_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud){

    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess=(init_translation*init_rotation).matrix();

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::string algorithm;
    this->get_parameter("algorithm",algorithm);

    if(algorithm=="gicp"){
        RCLCPP_INFO(this->get_logger(),"GICP wil be used!");
        gicp_registration(input_cloud,target_cloud,filtered_cloud,output_cloud);
    }else if(algorithm=="ndt"){
        RCLCPP_INFO(this->get_logger(),"NDT will be used!");
        ndt_registration(input_cloud,target_cloud,filtered_cloud,output_cloud,init_guess);
    }else{
        RCLCPP_ERROR(this->get_logger(),"Probably given algorithm is not correct you may want to use either ndt or gicp!");
    }

    std::string out_pcd_path;
    this->get_parameter("out_pcd_path",out_pcd_path);
    pcl::io::savePCDFileASCII(out_pcd_path, *output_cloud);

    pcd_coloring(target_cloud,input_cloud,output_cloud);
}

void PcdRegNode::pcd_coloring(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud){
    pcl::copyPointCloud(*target_cloud,this->target_cloudrgb);
    pcl::copyPointCloud(*input_cloud,this->input_cloudrgb);
    pcl::copyPointCloud(*output_cloud,this->output_cloudrgb);

    for(auto &p: this->target_cloudrgb.points){
        p.r=255;
        p.g=0;
        p.b=0;
    }
    for(auto &p: this->input_cloudrgb.points){
        p.r=0;
        p.g=255;
        p.b=0;
    }
    for(auto &p: this->output_cloudrgb.points){
        p.r=0;
        p.g=0;
        p.b=255;
    }
}

void PcdRegNode::pcd_pub(){
    auto in_cloud1=sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(this->target_cloudrgb, in_cloud1);
    in_cloud1.header.frame_id = "map";
    in_cloud1.header.stamp = now();

    auto in_cloud2=sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(this->input_cloudrgb, in_cloud2);
    in_cloud2.header.frame_id = "map";
    in_cloud2.header.stamp = now();

    auto transformed_cloud=sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(this->output_cloudrgb, transformed_cloud);
    transformed_cloud.header.frame_id = "map";
    transformed_cloud.header.stamp = now();

    in_pub1->publish(in_cloud1);
    in_pub2->publish(in_cloud2);
    out_pub->publish(transformed_cloud);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcdRegNode>());
    rclcpp::shutdown();
    return 0;
}