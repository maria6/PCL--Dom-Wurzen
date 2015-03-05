#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>
#include <vector>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <boost/filesystem.hpp>
#include "SACSegmentation.h"


SACSegmentation::SACSegmentation(void)
{}
SACSegmentation::~SACSegmentation(void)
{}

// output objects
std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> inliers_cloudvector;
std::vector<std::vector<double>> coefficients_vectorvector;


void SACSegmentation::UseSACSegmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, std::string  cloud_output_path)
{
	//////Planar Segmentation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_indices (new pcl::PointIndices);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.3); //Höchstabstand, bei dem ein Punkt noch zur Ebene gehören darf
	seg.setMaxIterations (1000);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	int i = 0, nr_points = (int) cloud_input->points.size ();
	// While 30% of the original cloud is still there
	while (cloud_input->points.size () > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_input);
		seg.segment (*inliers_indices, *coefficients);
		if (inliers_indices->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		//write coefficients into a single vector and this vector into another one that collects them
		std::vector<double> coefficients_vector;
		coefficients_vector.push_back(coefficients->values[0]);
		coefficients_vector.push_back(coefficients->values[1]);
		coefficients_vector.push_back(coefficients->values[2]);
		coefficients_vector.push_back(coefficients->values[3]);
		coefficients_vectorvector.push_back(coefficients_vector);

		// Extract the inliers_indices (= the points in the new plane)
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inliers_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		extract.setInputCloud (cloud_input);
		extract.setIndices (inliers_indices);
		extract.setNegative (false);
		extract.filter (*inliers_cloud);
		std::cerr << "PointCloud representing the planar component " << i << ": " << inliers_cloud->width * inliers_cloud->height << " data points." << std::endl;

		// write points to cloudvector to collect the pointclouds
		inliers_cloudvector.push_back(inliers_cloud);

		// create output folder if neccessary
		const char* path = cloud_output_path.c_str();
		boost::filesystem::path dir(path);
		if(boost::filesystem::create_directory(dir)) {
			std::cerr<< "Directory Created: "<<cloud_output_path<<std::endl;
		}

		// write points to file
		std::stringstream ss;
		pcl::PCDWriter writer;
		ss << "plane_" << i << ".pcd";
		writer.write<pcl::PointXYZRGBA> (cloud_output_path + ss.str (), *inliers_cloud, false);

		// Create the filtering object for next loop
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_input.swap (cloud_f);
		i++;
	}


	//Test Output for Plane Coefficients (it is a vector that contains one vector with coefficients per plane) 
	 std::cout << std::endl;
	 std::vector<double> coefficients_vector;
	 for (int i=0; i < coefficients_vectorvector.size(); i++){
	 	coefficients_vector = coefficients_vectorvector[i];
	 	std::cout<<"Coefficients for plane " << i << ": " << 
	 		coefficients_vector[0] << " " << coefficients_vector[1] << " " << coefficients_vector[2] << " " << coefficients_vector[3] << std::endl;
	 }
	 std::cout << std::endl;

	 //// test the size of each cloud in inliers_vector
	 //for (int j=0; j < inliers_cloudvector.size();j++){
	 //	std::cout <<inliers_cloudvector[j]->width * inliers_cloudvector[j]->height << std::endl;
	 //}
	 //std::cout << std::endl;

	std::cout<< "Number of Planes	        (=Length Inliers_CloudVector): "<< inliers_cloudvector.size() <<std::endl;
	std::cout<< "Number of Coefficient Sets (=Length Coefficients_Vector): "<< coefficients_vectorvector.size() <<std::endl;
	std::cout << std::endl;
}


std::vector<std::vector<double>> SACSegmentation::getCoefficients_Vector(){
	return coefficients_vectorvector;
}
	
std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> SACSegmentation::getInliers_CloudVector(){
	return inliers_cloudvector;
}

/*
Quellen:
	//SACSegmentation, um Punkte in einer Ebene zu finden
	//http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php
	//Danach Extract Indices, um diese Punkte in neue datei zu schreiben
	//http://pointclouds.org/documentation/tutorials/extract_indices.php
	*/