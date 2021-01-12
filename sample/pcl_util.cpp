#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl_util.hpp"
#include "FloatingObject.hpp"

float pcl_util::squareDist(const Eigen::Vector3f& p1, const pcl::PointXYZ p2) {
	return (p1.x() - p2.x) * (p1.x() - p2.x)
		+ (p1.y() - p2.y) * (p1.y() - p2.y)
		+ (p1.z() - p2.z) * (p1.z() - p2.z);
}

float pcl_util::squareDist(const pcl::PointXYZ p1, const Eigen::Vector3f& p2) {
	return (p1.x - p2.x()) * (p1.x - p2.x())
		+ (p1.y - p2.y()) * (p1.y - p2.y())
		+ (p1.z - p2.z()) * (p1.z - p2.z());
}

pcl_util::pcl_ptr pcl_util::passthrough(
	pcl_util::pcl_ptr cloud,
	const std::string& field_name,
	float limit_min,
	float limit_max
) {
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName(field_name);
	pass.setFilterLimits(limit_min, limit_max);
	pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pass.filter(*cloud_filtered);
	return cloud_filtered;
}

pcl_util::pcl_ptr pcl_util::TrimPointsOutsideWorkspace(
	dynaman::FloatingObjectPtr pObject,
	pcl_util::pcl_ptr pCloud
) {
	auto lb = pObject->lowerbound();
	auto ub = pObject->upperbound();
	auto radius = pObject->Radius() / 1000.f;
	auto pCloudFiltered = passthrough(pCloud, "x", -lb.x() - radius, lb.x() + radius);
	pCloudFiltered = passthrough(pCloudFiltered, "y", -lb.y() - radius, lb.y() + radius);
	pCloudFiltered = passthrough(pCloudFiltered, "z", -lb.z() - radius, lb.z() + radius);
	return pCloudFiltered;
}

std::vector<pcl::PointIndices> pcl_util::EuclidianClusterExtraction(
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
	double tol,
	int minClusterSize,
	int maxClusterSize
) {
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ecex;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	ecex.setSearchMethod(tree);
	ecex.setClusterTolerance(tol);
	ecex.setMinClusterSize(minClusterSize);
	ecex.setMaxClusterSize(maxClusterSize);
	ecex.setInputCloud(pCloud);
	std::vector<pcl::PointIndices> clusterIndices;
	ecex.extract(clusterIndices);
	return  clusterIndices;
}

pcl_util::pcl_ptr pcl_util::ExtractPointCloud(
	pcl_ptr pCloud,
	pcl::PointIndices pointIndices
) {
	pcl_ptr pCloudExtracted(new pcl::PointCloud<pcl::PointXYZ>());
	pCloudExtracted->points.resize(pointIndices.indices.size());
	pCloudExtracted->width = pointIndices.indices.size();
	pCloudExtracted->height = 1;
	pCloudExtracted->is_dense = true;
	for(int idx_pt = 0; idx_pt < pointIndices.indices.size(); idx_pt++){
		pCloudExtracted->points[idx_pt] = pCloud->points[pointIndices.indices[idx_pt]];
	}
	return pCloudExtracted;
}
