#include "Visualizer.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

	
int user_data;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
std::string path ="";

Visualizer::Visualizer(void)
{	
}
Visualizer::~Visualizer(void)
{
}


void viewerOneOff (pcl::visualization::PCLVisualizer& viewer){
    viewer.setBackgroundColor (1, 1, 1);
    //pcl::PointXYZ o;
    //o.x = 1.0;
    //o.y = 0;
    //o.z = 0;
    //viewer.addSphere (o, 0.25, "sphere", 0);
	
    //std::cout << "i only run once" << std::endl;    
}

//chosen cloud
void Visualizer::showCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2)
{
	cloud1.swap(cloud2);
	std::cout << "loading Viewer..." << std::endl;
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud1);
	
 
    //use the following functions to get access to the underlying more advanced/powerful
    ////PCLVisualizer   
    //This will only get called once
    viewer.runOnVisualizationThread(viewerOneOff);
    //This will get called once per visualization iteration
    //viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    user_data++;
    }
}