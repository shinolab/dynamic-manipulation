#include <utility>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "FloatingObject.hpp"
#include "HandStateReader.hpp"

using namespace dynaman;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

PclHandStateReader::PclHandStateReader(
	dynaman::FloatingObjectPtr pObject,
	std::shared_ptr<pcl_grabber> pPclGrabber)
	:m_pObject(pObject)
	,m_PclGrabber(pPclGrabber){}

PclHandStateReader::~PclHandStateReader() {};

std::shared_ptr<HandStateReader> PclHandStateReader::Create(
	dynaman::FloatingObjectPtr pObject,
	std::shared_ptr<pcl_grabber> pPclGrabber
) {
	return std::make_shared<PclHandStateReader>(pObject, pPclGrabber);
}

bool PclHandStateReader::Read(HandState& state) {
	return true;
}

