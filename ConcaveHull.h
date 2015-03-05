#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <string>

class ConcaveHull
{
public:
	ConcaveHull(void);
	~ConcaveHull(void);
	void Hull(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cloudvector_in, std::string  cloud_output_path);
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> getOutput();
};


