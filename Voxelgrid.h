#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>

class Voxelgrid
{
public:
	Voxelgrid(void);
	~Voxelgrid(void);
	void ReducePixel(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, std::string  cloud_output_path, float x, float y, float z);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getOutput();
};


