//#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>

class Visualizer
{
public:
	Visualizer(void);
	~Visualizer(void);
	void showCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
};
