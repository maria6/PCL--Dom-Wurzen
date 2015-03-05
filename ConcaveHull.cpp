#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/concave_hull.h>
#include <boost/filesystem.hpp>
#include "ConcaveHull.h"


ConcaveHull::ConcaveHull(void)
{	
}
ConcaveHull::~ConcaveHull(void)
{
}

//Output Cloudvector
std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> hull_cloudvector_out;

void ConcaveHull::Hull(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cloudvector_in, std::string cloud_output_path)
{
  for (int i=0; i<cloudvector_in.size(); i++){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in;
    cloud_in.swap(cloudvector_in[i]);
    // Create a Concave Hull
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::ConcaveHull<pcl::PointXYZRGBA> chull;
    chull.setInputCloud (cloud_in);
    chull.setDimension(2);
    // Set alpha, which is the maximum length from a vertex to the center of the voronoi cell
    // (the smaller, the greater the resolution of the hull).
    chull.setAlpha (1);
    chull.reconstruct (*cloud_hull);
    
    std::cerr << "Concave hull "<< i << " has: " << cloud_hull->points.size ()
              << " data points." << std::endl;

    // create output folder if neccessary
    const char* path = cloud_output_path.c_str();
    boost::filesystem::path dir(path);
    if(boost::filesystem::create_directory(dir)) {
      std::cerr<< "Directory Created: "<<cloud_output_path<<std::endl;
    }

    // write points to file
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss << "concave_hull_" << i << ".pcd";
    writer.write<pcl::PointXYZRGBA> (cloud_output_path + ss.str (), *cloud_hull, false); //*
    
    hull_cloudvector_out.push_back(cloud_hull);
  }
};

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> ConcaveHull::getOutput(){
	return hull_cloudvector_out;
}
