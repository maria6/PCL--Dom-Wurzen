#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include "Voxelgrid.h"


Voxelgrid::Voxelgrid(void)
{	
}
Voxelgrid::~Voxelgrid(void)
{
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr VG_cloud_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr VG_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);

void Voxelgrid::ReducePixel(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, std::string cloud_output_path, float x, float y, float z)
{
	VG_cloud_in.swap(cloud_input);

	std::cerr << "PointCloud before filtering: " << VG_cloud_in->width * VG_cloud_in->height 
	 << " data points (" << pcl::getFieldsList (*VG_cloud_in) << ")." << std::endl;

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGBA>  sor;
	sor.setInputCloud (VG_cloud_in);
	sor.setLeafSize (x,y,z);	//zu erstellende Voxelgröße in Meter. ggf anpassen
	std::cerr<<"Used LeafSize in Meter: x,y,z:"<< x <<","<< y <<","<< z << "." << std::endl;
	sor.filter (*VG_cloud_filtered);

	std::cerr << "PointCloud after filtering: " << VG_cloud_filtered->width * VG_cloud_filtered->height 
	 << " data points (" << pcl::getFieldsList (*VG_cloud_filtered) << ")." << std::endl;

	pcl::PCDWriter writer;
	writer.write (cloud_output_path, *VG_cloud_filtered, false);
	std::cout << "PointCloud saved to File: " << cloud_output_path << std::endl;
};

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Voxelgrid::getOutput(){
	return VG_cloud_filtered;
}




