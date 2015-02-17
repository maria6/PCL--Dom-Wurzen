#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <string>

class NormalEstimation
{
public:
	NormalEstimation(void);
	~NormalEstimation(void);
	void UseNormalEstimation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, std::string cloud_output_path, int RadiusSearch);
	pcl::PointCloud<pcl::Normal>::Ptr getOutput();
};