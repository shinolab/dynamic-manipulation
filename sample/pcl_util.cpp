#include <pcl/filters/passthrough.h>
#include "pcl_util.hpp"
#include "FloatingObject.hpp"

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