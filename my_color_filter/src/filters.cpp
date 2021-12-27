#include "filters.h"
#include <pcl/point_types_conversion.h>
#include <pcl/filters/extract_indices.h>


pcl::PCLPointCloud2Ptr red_extraction(pcl::PCLPointCloud2ConstPtr& cloudin)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2 (*cloudin, *cloudin2);

	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
	pcl::PointCloudXYZRGBtoXYZHSV(*cloudin2, *cloud_HSV);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr redout(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	for (int i = 0; i < cloudin2->points.size(); i++)
	{
		if ((cloud_HSV->points[i].h < 30 || cloud_HSV->points[i].h > 330) && (cloud_HSV->points[i].s > 0.5) && (cloud_HSV->points[i].v > 0.5))
		{
			inliers->indices.push_back(i);
		}
	}
	pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true);
	eifilter.setInputCloud(cloudin2);
	eifilter.setIndices(inliers);
	eifilter.filter(*redout);

    pcl::PCLPointCloud2Ptr redout2 (new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2 (*redout, *redout2);

	return redout2;
}

pcl::PCLPointCloud2Ptr voxelGrid(const pcl::PCLPointCloud2Ptr& in, float sz) {
	pcl::PCLPointCloud2Ptr out (new pcl::PCLPointCloud2);
	pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
	vg.setInputCloud(in);
	vg.setLeafSize(sz,sz,sz);
	vg.filter(*out);
	return out;
}

pcl::PCLPointCloud2Ptr removeOutliers(const pcl::PCLPointCloud2Ptr& in) {
	pcl::PCLPointCloud2Ptr out (new pcl::PCLPointCloud2);
	pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(in);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*out);
	return out;
}