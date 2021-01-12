#ifndef _DYNAMAN_PCL_UTIL_HPP
#define _DYNAMAN_PCL_UTIL_HPP

#include <string>
#include <utility>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "FloatingObject.hpp"

namespace pcl_util {
	using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

	float squareDist(const Eigen::Vector3f& p1, const pcl::PointXYZ p2);

	float squareDist(const pcl::PointXYZ p1, const Eigen::Vector3f& p2);

	pcl_ptr passthrough(
		pcl_ptr cloud,
		const std::string& field_name,
		float limit_min,
		float limit_max
	);

	pcl_ptr TrimPointsOutsideWorkspace(dynaman::FloatingObjectPtr pObject, pcl_ptr pCloud);

	std::vector<pcl::PointIndices> EuclidianClusterExtraction(
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
		double tol,
		int minClusterSize,
		int maxClusterSize
	);

	pcl_ptr ExtractPointCloud(
		pcl_ptr pCloud,
		pcl::PointIndices pointIndices
	);
}

#endif // !_DYNAMAN_PCL_UTIL_HPP
