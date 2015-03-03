#include "Start.h"
#include "Visualizer.h"
#include "Voxelgrid.h"
#include "StatisticalOutlierRemoval.h"
#include "NormalEstimation.h"
#include "SACSegmentation.h"

#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>




int main (int argc, char** argv)
{	// Create some empty pointclouds
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZRGBA>);

	// Set home as folder for point cloud data 
	std::string home=("D:\\Dokumente\\Studium\\Wahlfaecher\\Objekterkennung und Geodatenfusion\\Test01\\Punktwolken\\");
	// Set input data if available
	std::string input=(home+"Dom_filtered_0,1m.txt");  //Set Path
	std::string filter_output=(home+"Dom_filtered_1m.txt");//Set Path
	std::string filter_output_inliers=(home+"Dom_filtered_1m_inliers.txt"); //Set Path
	std::string filter_output_outliers=(home+"Dom_filtered_1m_outliers.txt"); //Set Path
	std::string normals_output=(home+"Dom_filtered_1m_normals.txt");//Set Path
	std::string segmentation_output_folder=(home+"Segmentation\\");

	// Read point cloud data from paths into empty point clouds 
	pcl::PCDReader reader;
	Visualizer visualizer01;
	reader.read (input, *cloud1); 
	reader.read (filter_output, *cloud2);
	reader.read (filter_output_inliers, *cloud3); 	
	reader.read (filter_output_outliers, *cloud4); 

	// // Reduce Number of Pixel
	// Voxelgrid Voxel;
	// //Input: (InputPointCloud [cloud], OutputPath [string], Voxelsize for x,y,z in m [float])
	// Voxel.ReducePixel(cloud1, filter_outputneu, 1.0f, 1.0f, 1.0f); //Set Voxel Size here
	// //Show Reduced Pointcloud
	// visualizer01.showCloud(Voxel.getOutput());

	////Remove Outlier
	//StatisticalOutlierRemoval Sor;
	//// Input: (InputPointCloud [cloud], OutputPath1 [string],  OutputPath2 [string], NumberOfNeighborPoints [int], StandardDeviation [double])
	//Sor.UseOutlierRemoval(Voxel.getOutput(),filter_output_inliers,filter_output_outliers, 50, 1.0); //Set Vaulues here
	////Show Outliers after Filtering
	//visualizer01.showCloud(Sor.getOutputOutlier());
	////Show Inliers after Filtering
	//visualizer01.showCloud(Sor.getOutputInlier());

	// // Find Normals with NormalEstimation
	//NormalEstimation Nes;
	//Nes.UseNormalEstimation(cloud1, normals_output, 5);
	//visualizer01.showCloud(Nes.getOutput());   ---> Normalen nicht mit Cloudviewer darstellbar!!!!!
	

	//wait for Enter, then quit	
	std::cin.ignore();	//std::cin.get(); //system("PAUSE"); //no difference

	return (0);
}



