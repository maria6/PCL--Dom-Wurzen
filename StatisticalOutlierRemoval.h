#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class StatisticalOutlierRemoval
{
public:
	StatisticalOutlierRemoval(void);
	~StatisticalOutlierRemoval(void);
	void UseOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, std::string cloud_output_path_inlier,std::string cloud_output_path_outlier, int MeanK, double StddevMulThresh);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getOutputInlier();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getOutputOutlier();
};