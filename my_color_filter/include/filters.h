#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

pcl::PCLPointCloud2Ptr red_extraction(pcl::PCLPointCloud2ConstPtr& cloudin);

pcl::PCLPointCloud2Ptr voxelGrid(const pcl::PCLPointCloud2Ptr& in, float sz);

pcl::PCLPointCloud2Ptr removeOutliers(const pcl::PCLPointCloud2Ptr& in);