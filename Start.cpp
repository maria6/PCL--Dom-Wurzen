#include "Start.h"
#include "Visualizer.h"
#include "Voxelgrid.h"
#include "StatisticalOutlierRemoval.h"
#include "NormalEstimation.h"
#include "SACSegmentation.h"
#include "Clustering.h"
#include "ConcaveHull.h"


#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/pcl_macros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>




int main (int argc, char** argv)
{	// Create some empty pointclouds
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);

	// Set home as folder for point cloud data 
	std::string home=("D:\\Dokumente\\Studium\\Wahlfaecher\\Objekterkennung und Geodatenfusion\\Test01\\Punktwolken\\");
	// Set input data if available
	std::string input=(home+"Dom_filtered_0,1m.txt");  //Set Path
	std::string filter_output=(home+"Dom_filtered_1m.txt");//Set Path
	std::string filter_output_inliers=(home+"Dom_filtered_1m_inliers.txt"); //Set Path
	std::string filter_output_outliers=(home+"Dom_filtered_1m_outliers.txt"); //Set Path
	std::string normals_output=(home+"Dom_filtered_1m_normals.txt");//Set Path
	std::string segmentation_output_folder=(home+"Segmentation\\");
	std::string clustering_output_folder(home+"Clustering\\");
	std::string concavehull_output_folder(home+"ConcaveHull\\");

	// Read point cloud data from paths into empty point clouds 
	pcl::PCDReader reader;
	Visualizer visualizer01;
	reader.read (input, *cloud1); 
	reader.read (filter_output, *cloud2);
	
	visualizer01.showCloud(cloud1);
	visualizer01.showCloud(cloud2);

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
	

	 // Find Planes in the Pointcloud with SACSegmentation
	 SACSegmentation seg;
	 seg.UseSACSegmentation(cloud2, segmentation_output_folder);

	 //// Read plane clouds from returned cloud vector
	 //for (int i=0; i < seg.getInliers_CloudVector().size(); i++){
	 //	visualizer01.showCloud(seg.getInliers_CloudVector()[i]);
	 //}

	 ////Get the Plane Coefficients and write them to console (it aktually is a vector that contains one vector with coefficients per plane) 
	 //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	 //std::vector<double> coefficients_vector;
	 //for (int i=0; i < seg.getCoefficients_Vector().size(); i++){
	 //	coefficients_vector = seg.getCoefficients_Vector()[i];
	 //	std::cout<<" Coefficients for plane " << i << ": " << coefficients_vector[0] << " " << coefficients_vector[1] << " " << coefficients_vector[2] << " " << coefficients_vector[3] << std::endl;
	 //}



	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clouda (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudb (new pcl::PointCloud<pcl::PointXYZRGBA>);
	 // Read plane clouds from returned cloud vector
	 	clouda = seg.getInliers_CloudVector()[0];
		cloudb = seg.getInliers_CloudVector()[1];

	 //Get the Plane Coefficients and write them to console (it aktually is a vector that contains one vector with coefficients per plane) 
	 //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	 pcl::visualization::PCLVisualizer viewer;
	 std::vector<double> coefficients_vector1;
	 std::vector<double> coefficients_vector2;
	 pcl::ModelCoefficients coeffs;
	 	coefficients_vector1 = seg.getCoefficients_Vector()[0];
		coefficients_vector2 = seg.getCoefficients_Vector()[1];
	 	//std::cout<<" Coefficients for plane " << i << ": " << coefficients_vector[0] << " " << coefficients_vector[1] << " " << coefficients_vector[2] << " " << coefficients_vector[3] << std::endl;
		

			double angular_tolerance=0;
Eigen::Vector4f plane_a;
plane_a.x()=coefficients_vector1[0];
plane_a.y()=coefficients_vector1[1];
plane_a.z()=coefficients_vector1[2];
plane_a.w()=coefficients_vector1[3];
Eigen::Vector4f plane_b;
plane_b.x()=coefficients_vector2[0];
plane_b.y()=coefficients_vector2[1];
plane_b.z()=coefficients_vector2[2];
plane_b.w()=coefficients_vector2[3];

Eigen::VectorXf line;
pcl:: planeWithPlaneIntersection(plane_a,plane_b,line,angular_tolerance);


pcl::ModelCoefficients::Ptr l(new pcl::ModelCoefficients ());
l->values.resize(6);
for (int i=0;i<5;i++)
{
l->values[i]=line[i];
}


viewer.addPointCloud(clouda); 
//viewer.addPointCloud(cloudb);
viewer.addLine(*l);
viewer.resetCamera();

		



	 ////Find Cluster in the Planes
	 //Clustering cluster;
	 //cluster.Extraction(seg.getInliers_CloudVector(), clustering_output_folder);
	 //// Read plane clouds from returned cloud vector
	 //for (int i=0; i < cluster.getOutput().size(); i++){
	 //	 visualizer01.showCloud(cluster.getOutput()[i]);
	 //}
	

	 //// Find outline of clouds
	 //ConcaveHull ch;
	 //ch.Hull(cluster.getOutput(), concavehull_output_folder);
	 //// Read hull clouds from returned cloud vector
	 //for (int i=0; i < cluster.getOutput().size(); i++){
	 //	 visualizer01.showCloud(ch.getOutput()[i]);
	 //}
	 



	//wait for Enter, then quit	
	std::cin.ignore();	//std::cin.get(); //system("PAUSE"); //no difference

	return (0);
}



