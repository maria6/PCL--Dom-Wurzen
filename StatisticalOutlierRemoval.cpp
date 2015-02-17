#include "StatisticalOutlierRemoval.h"

StatisticalOutlierRemoval::StatisticalOutlierRemoval(void)
{
}
StatisticalOutlierRemoval::~StatisticalOutlierRemoval(void)
{
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SOR_cloud_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SOR_cloud_filtered_inlier(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SOR_cloud_filtered_outlier(new pcl::PointCloud<pcl::PointXYZRGBA>);

void StatisticalOutlierRemoval::UseOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, std::string cloud_output_path_inlier,std::string cloud_output_path_outlier, int MeanK, double StddevMulThresh)
{
	SOR_cloud_in.swap(cloud_input);
	
	// Create the filtering object
	std::cerr << "Using StatisticalOutlierRemoval..." << std::endl;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
	sor.setInputCloud (SOR_cloud_in);
	sor.setMeanK (MeanK);
	sor.setStddevMulThresh (StddevMulThresh);
	sor.filter (*SOR_cloud_filtered_inlier);

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGBA> (cloud_output_path_inlier, *SOR_cloud_filtered_inlier, false);
	std::cout << "PointCloud_Inlier saved to File: " << cloud_output_path_inlier << std::endl;

	sor.setNegative (true);
	sor.filter (*SOR_cloud_filtered_outlier);
	writer.write<pcl::PointXYZRGBA> (cloud_output_path_outlier, *SOR_cloud_filtered_outlier, false);
	std::cout << "PointCloud_Outlier saved to File: " << cloud_output_path_outlier << std::endl;

}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr StatisticalOutlierRemoval::getOutputInlier()
{ return SOR_cloud_filtered_inlier;
}
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr StatisticalOutlierRemoval::getOutputOutlier()
{ return SOR_cloud_filtered_outlier;
}