#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include "Clustering.h"

Clustering::Clustering(void)
{}
Clustering::~Clustering(void)
{}

 
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr KT_cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>);

 void Clustering::Extraction(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr KT_cloud_in, std::string cloud_output_path)
  {

// Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  tree->setInputCloud (KT_cloud_in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  ec.setClusterTolerance (1.1); //in meter
  ec.setMinClusterSize (100);  //Number of Points
  ec.setMaxClusterSize (25000); //Number of Points
  ec.setSearchMethod (tree);
  ec.setInputCloud (KT_cloud_in);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    cloud_cluster->points.push_back (KT_cloud_in->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

	  pcl::PCDWriter writer;
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZRGBA> (cloud_output_path + ss.str (), *cloud_cluster, false); //*
    KT_cloud_out.swap(cloud_cluster);
    j++;
  }

};

// Bisher nur einzelne Wolke als Output, Rest fehlt!
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Clustering::getOutput(){
  return KT_cloud_out;
}