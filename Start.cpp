#include "Start.h"
#include "Visualizer.h"
#include "Voxelgrid.h"
#include "StatisticalOutlierRemoval.h"
#include "NormalEstimation.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


int main (int argc, char** argv)
{	//empty pointclouds
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);

	// set home as folder for point cloud data
	std::string home=("D:\\Dokumente\\Studium\\Wahlfaecher\\Objekterkennung und Geodatenfusion\\Test01\\Punktwolken\\");
	std::string input=(home+"Dom_filtered_0,1m.txt");  //Set Path
	std::string filter_output=(home+"Dom_filtered_1m.txt");//Set Path
	std::string filter_output_inliers=(home+"Dom_filtered_1m_inliers.txt"); //Set Path
	std::string filter_output_outliers=(home+"Dom_filtered_1m_outliers.txt"); //Set Path
	std::string normals_output=(home+"Dom_filtered_1m_normals.txt");//Set Path

	//read point cloud data from paths into empty point clouds
	//filter+outlier müssen ggf erst mit unten auskommentierten funktionen erstellt werden! 
	pcl::PCDReader reader;
	Visualizer blubb01;
	reader.read (input, *cloud1); 
	reader.read (filter_output, *cloud2);
	reader.read (filter_output_inliers, *cloud3); 	
	reader.read (filter_output_outliers, *cloud4); 

	////Reduce Number of Pixel
	//Voxelgrid Voxel;
	////Input: (InputPointCloud [cloud], OutputPath [string], Voxelsize for x,y,z in m [float])
	//Voxel.ReducePixel(cloud, filter_output, 1.0f, 1.0f, 1.0f); //Set Voxel Size here

	////Show Reduced Pointcloud
	//Visualizer blubb02;
	//blubb02.showCloud(Voxel.getOutput());

	////Remove Outlier
	//StatisticalOutlierRemoval Sor;
	//// Input: (InputPointCloud [cloud], OutputPath1 [string],  OutputPath2 [string], NumberOfNeighborPoints [int], StandardDeviation [double])
	//Sor.UseOutlierRemoval(Voxel.getOutput(),filter_output_inliers,filter_output_outliers, 50, 1.0); //Set Vaulues here

	////Show Outliers after Filtering
	//Visualizer blubb03;
	//blubb03.showCloud(Sor.getOutputOutlier());
	
	////Show Inliers after Filtering
	//Visualizer blubb04;
	//blubb04.showCloud(Sor.getOutputInlier());


	//_________Funktion für unten geschilderten Ablauf vermutlich nicht benötigt
	//NormalEstimation Nes;
	//Nes.UseNormalEstimation(cloud1, normals_output, 5);
	//blubb01.showCloud(Nes.getOutput());   ---> Normalen nicht mit Cloudviewer darstellbar!!!!!


	blubb01.showCloud(cloud2);







	//SACSegmentation, um Punkte in einer Ebene zu finden
	//http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php
	//Danach Extract Indices, um diese Punkte in neue datei zu schreiben
	//http://pointclouds.org/documentation/tutorials/extract_indices.php

	////////////////////////
	// - spielen mit Treshhold und %, bis zu denen iteriert werden soll (SAGSegmentation+ExtractIndices)
	// - ausreißerfilter nach dem ebenenfinden ist sicher sinnvoll, da fremde objekte in ebenen liegen
	// - oder evtl methode die sicherstellt, dass ebene aus einem objekt besteht = segmentierung?
	// - es muss aus der Ebenenpunktwolke noch ein polygon erstellt werden (coefficients rausschreiben)
	// --> äußerste Punkte der ebene als "außenkante" finden. diese könnte als begrenzung für polygon dienen, wenn es es aus koeffizienten generiert wird
	// ==> siehe Bernd Grafes Anleitung in Präsentation 
	// - schleife in main, die jeweils in einem durchlauf folgendes machen lässt:
	// - finde eine ebene, extrahiere die punkte 
	// - bereinigen und außenkante finden
	// - polygon erstellen (+bearbeiten? und speichern)
	// - nächste Ebene finden
	// -->alle methoden als separate klassen erstellen?
	//
	/////////////////////////

	//////Planar Secmentation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
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

	int i = 0, nr_points = (int) cloud2->points.size ();
	// While 40% of the original cloud is still there
	while (cloud2->points.size () > 0.4 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud2);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		////Ausgabe der Ebenenkoeffizienten (in ax + by + cz + d = 0 form)
		//std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
		//	<< coefficients->values[1] << " "
		//	<< coefficients->values[2] << " " 
		//	<< coefficients->values[3] << std::endl;

		// Extract the inliers (= the points in the new plane)
		extract.setInputCloud (cloud2);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_p);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		//write points to file (in .exe folder)
		std::stringstream ss;
		pcl::PCDWriter writer;
		ss << "plane_"  <<  cloud_p->width * cloud_p->height << "_" << i << ".pcd";
		writer.write<pcl::PointXYZRGBA> (ss.str (), *cloud_p, false);

		// Create the filtering object
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud2.swap (cloud_f);
		i++;
	}


	//wait for Enter, then quit
	//std::cin.get();
	std::cin.ignore();	//no differece






	return (0);
}



