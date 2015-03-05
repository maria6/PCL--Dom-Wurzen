#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vector>

class SACSegmentation
{
public:
	SACSegmentation(void);
	~SACSegmentation(void);
	void UseSACSegmentation(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, std::string  cloud_output_path);
	std::vector<std::vector<double>> getCoefficients_Vector();
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> getInliers_CloudVector();
};
