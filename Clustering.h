#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <string>

class Clustering
{
public:
	Clustering(void);
	~Clustering(void);
	void Extraction(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cloudvector_in, std::string  cloud_output_path);
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> getOutput();
};


