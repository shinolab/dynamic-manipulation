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
	auto lb = 0.001f * pObject->lowerbound();
	auto ub = 0.001f * pObject->upperbound();
	auto radius = 0.001f * pObject->Radius();
	auto pCloudFiltered = passthrough(pCloud, "x", lb.x() - radius, ub.x() + radius);
	pCloudFiltered = passthrough(pCloudFiltered, "y", lb.y() - radius, ub.y() + radius);
	pCloudFiltered = passthrough(pCloudFiltered, "z", lb.z() - radius, ub.z() + radius);
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

pcl_util::pcl_ptr pcl_util::MakeSphere(
	const Eigen::Vector3f& center,
	float radius
) {
	int num_long = 50;
	int num_lat = 25;
	int num_points = num_long * num_lat;
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(num_points);
	cloud->height = num_lat;
	cloud->width = num_long;
	for (int i_lat = 0; i_lat < num_lat; i_lat++)
	{
		for (int i_long = 0; i_long < num_long; i_long++) {
			int i_vert = i_lat * num_long + i_long;
			float theta = M_PI * i_lat / num_lat;
			float phi = 2 * M_PI * i_long / num_long;
			cloud->points[i_vert].x = center.x() + radius * sin(theta) * cos(phi);
			cloud->points[i_vert].y = center.y() + radius * sin(theta) * sin(phi);
			cloud->points[i_vert].z = center.z() + radius * cos(theta);
		}
	}
	return cloud;
}

pcl_util::pcl_ptr pcl_util::EigenToPcl(
	const std::vector<Eigen::Vector3f>& points
) {
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = points.size();
	cloud->height = 1;
	cloud->is_dense = true;
	cloud->points.resize(points.size());
	for (int iv = 0; iv < points.size(); iv++) {
		cloud->points[iv].x = points[iv].x();
		cloud->points[iv].y = points[iv].y();
		cloud->points[iv].z = points[iv].z();
	}
	return cloud;
}
