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

// temp clouds for segmentation/extraction loop
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inliers_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);

// temp objects
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers_indices (new pcl::PointIndices);

// output objects
std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> inliers_cloudvector;
std::vector<double> coefficients_vector;
std::vector<std::vector<double>> coefficients_vectorvector;


void SACSegmentation::UseSACSegmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, std::string  cloud_output_path)
{
	//////Planar Segmentation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_indices (new pcl::PointIndices);
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
		coefficients_vector.push_back(coefficients->values[0]);
		coefficients_vector.push_back(coefficients->values[1]);
		coefficients_vector.push_back(coefficients->values[2]);
		coefficients_vector.push_back(coefficients->values[3]);
/////////////////////////////////////////////////////////////////////////////////
		// gleiches Problem wie bei CLoudvector. Vector enthält nur Cloud/Koeffizienten des letzten Schleifendurchlaufs
		coefficients_vectorvector.push_back(coefficients_vector);
///////////////////////////////////////////////////////////////////////////////

		////Ausgabe der Ebenenkoeffizienten in Konsole (in ax + by + cz + d = 0 form)
		//std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
		//	<< coefficients->values[1] << " "
		//	<< coefficients->values[2] << " " 
		//	<< coefficients->values[3] << std::endl;

		// Extract the inliers_indices (= the points in the new plane)
		extract.setInputCloud (cloud_input);
		extract.setIndices (inliers_indices);
		extract.setNegative (false);
		extract.filter (*inliers_cloud);
		std::cerr << "PointCloud representing the planar component: " << inliers_cloud->width * inliers_cloud->height << " data points." << std::endl;

///////////////////////////////////////////////////////////////	///////
		//Problembereich: 
		//Pro schleifendurchlauf soll die aktuelle Ebene (=inliers_cloud) in den vector (=inliers_cloudvector) geschrieben werden.
		//Der Vektor soll als Container dienen, um alle gefundenen Ebenen mit einem mal an die Main zurückgeben zu können.
		//Zuerst wird die aktuelle Ebene in den Vektor geschrieben, und mit der For-Schleife wird dann geprüft, wie groß die einzelnen Wolken im Vektor sind.
		//Nach Abschluss stimmt zwar auch die Anzahl der Ebenen im Vektor. Aber an jeder Stelle im Vektor liegt nur die allerletzte gefundene Ebene, anstatt erste bis letzte Ebene.
		//Es wirkt, als würde bei jedem Schleifendurchlauf der Platz im Vektor geschaffen werden, aber die konkrete Werteübergabe würde erst ganz am Ende geschehen.
		//Wie sorge ich also dafür, dass wirklich die jeweils aktuelle Wolke inliers_cloud im Vektor gespeichert wird und nicht nur eine Referenz, die am ja Ende des Durchlaufs nicht mehr stimmt?

		// write points to cloudvector to collect the pointclouds
		inliers_cloudvector.push_back(inliers_cloud);
		// test the size of each cloud in vector --> siehe output-screenshot
		for (int j=0; j < inliers_cloudvector.size();j++){
			std::cout <<inliers_cloudvector[j]->width * inliers_cloudvector[j]->height << std::endl;
		}
///////////////////////////////////////////////////////////////////////	

		// create output folder if neccessary
		const char* path = cloud_output_path.c_str();
		boost::filesystem::path dir(path);
		if(boost::filesystem::create_directory(dir)) {
			std::cerr<< "Directory Created: "<<cloud_output_path<<std::endl;
		}

		// write points to file
		std::stringstream ss;
		pcl::PCDWriter writer;
		ss << "plane_" << i << "_" <<  inliers_cloud->width * inliers_cloud->height << ".pcd";
		writer.write<pcl::PointXYZRGBA> (cloud_output_path + ss.str (), *inliers_cloud, false);


		// Create the filtering object for next loop
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_input.swap (cloud_f);
		i++;
	}


	std::cout<< "Number of Planes	        (=Length Inliers_CloudVector): "<< inliers_cloudvector.size() <<std::endl;
//	std::cout<< "Number of Coefficient Sets (=Length Coefficients_Vector): "<< coefficients_vectorvector.size() <<std::endl;

	

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