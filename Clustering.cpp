#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/filesystem.hpp>
#include "Clustering.h"

Clustering::Clustering(void)
{}
Clustering::~Clustering(void)
{}

 // Output Cloudvector  
 std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cloudvector_out;

 void Clustering::Extraction(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cloudvector_in, std::string cloud_output_path)
 {
  int j = 0;
  for (int i=0; i<cloudvector_in.size(); i++){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in;
    cloud_in.swap(cloudvector_in[i]);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud (cloud_in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance (1.1); //in meter
    ec.setMinClusterSize (100);  //Number of Points
    ec.setMaxClusterSize (25000); //Number of Points
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_in);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      // create output folder if neccessary
      const char* path = cloud_output_path.c_str();
      boost::filesystem::path dir(path);
      if(boost::filesystem::create_directory(dir)) {
        std::cerr<< "Directory Created: "<<cloud_output_path<<std::endl;
      }

      //write points to file
  	  pcl::PCDWriter writer;
      std::cout << "PointCloud representing the Cluster " << j << ": " << cloud_cluster->points.size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZRGBA> (cloud_output_path + ss.str (), *cloud_cluster, false); //*
      cloudvector_out.push_back(cloud_cluster);
      j++;
    }
  }  

};


std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> Clustering::getOutput(){
  return cloudvector_out;
}