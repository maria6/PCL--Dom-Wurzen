#include "NormalEstimation.h"

NormalEstimation::NormalEstimation(void)
{
}
NormalEstimation::~NormalEstimation(void)
{
}

// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr NE_cloud_normals (new pcl::PointCloud<pcl::Normal>);

void NormalEstimation::UseNormalEstimation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, std::string cloud_output_path, int RadiusSearch)
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	ne.setInputCloud (cloud_input);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
	ne.setSearchMethod (tree);

	// Use all neighbors in a sphere of radius in m
	ne.setRadiusSearch (RadiusSearch);
	std::cerr << "Computing Normals for Points in Radius of: " << RadiusSearch <<"m." << std::endl;
	// Compute the features
	ne.compute (*NE_cloud_normals);

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

	pcl::PCDWriter writer;
	writer.write (cloud_output_path, *NE_cloud_normals, false);
	std::cout << "Normals saved to File: " << cloud_output_path << std::endl;

};

pcl::PointCloud<pcl::Normal>::Ptr NormalEstimation::getOutput(){
	return NE_cloud_normals;
}