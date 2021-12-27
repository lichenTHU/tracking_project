#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include "filters.h"

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 

  // Convert to pcl::PCLPointCloud2 data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

  //构建新的指针cloudPtr2,存储red_extraction后的点云
  pcl::PCLPointCloud2Ptr cloudPtr2 = red_extraction(cloudPtr);

  //构建新的指针cloudPtr3,去除离群点＋voxel_grid滤波
  pcl::PCLPointCloud2Ptr cloudPtr3 = voxelGrid(removeOutliers(cloudPtr2), 0.001);

  // Convert to ROS data type,并设置voxel大小
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(*cloudPtr3, output);

  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_color_filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin();
}