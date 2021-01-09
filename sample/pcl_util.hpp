#ifndef _DYNAMAN_PCL_UTIL_HPP
#define _DYNAMAN_PCL_UTIL_HPP

#include <string>
#include <utility>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "FloatingObject.hpp"

namespace pcl_util {
	using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

	pcl_ptr passthrough(
		pcl_ptr cloud,
		const std::string& field_name,
		float limit_min,
		float limit_max
	);

	pcl_ptr TrimPointsOutsideWorkspace(dynaman::FloatingObjectPtr pObject, pcl_ptr pCloud);
}

#endif // !_DYNAMAN_PCL_UTIL_HPP
