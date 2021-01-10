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

namespace {
	const float thickness_collider_contact = 0.03;
	const float thickness_collider_click = 0.05;
	const float tol_cluster_dist = 0.05f;
	const int min_cluster_size_balloon = 100;
	const int max_cluster_size = 10000;
	const int thres_contact_num = 150;
}

using namespace dynaman;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

PclHandStateReader::PclHandStateReader(
	dynaman::FloatingObjectPtr pObject,
	std::shared_ptr<pcl_grabber> pPclGrabber)
	:m_pObject(pObject),
	m_pPclGrabber(pPclGrabber),
	m_radiusObject(pObject->Radius()),
	m_radiusColliderContact(pObject->Radius() + thickness_collider_contact),
	m_radiusColliderClick(pObject->Radius() + thickness_collider_contact + thickness_collider_click)
{}

PclHandStateReader::~PclHandStateReader() {};

std::shared_ptr<HandStateReader> PclHandStateReader::Create(
	dynaman::FloatingObjectPtr pObject,
	std::shared_ptr<pcl_grabber> pPclGrabber
) {
	return std::make_shared<PclHandStateReader>(pObject, pPclGrabber);
}

pcl_util::pcl_ptr PclHandStateReader::DefaultPreprocess(pcl_util::pcl_ptr pCloud) {
	auto pCloudInside = pcl_util::TrimPointsOutsideWorkspace(m_pObject, pCloud);
	pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
	vg_filter.setInputCloud(pCloudInside);
	vg_filter.setLeafSize(0.005f, 0.005f, 0.005f);
	pcl_ptr pCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>());
	vg_filter.filter(*pCloudFiltered);
	return pCloudFiltered;
}

bool PclHandStateReader::Initialize() {
	auto pCloudRaw = m_pPclGrabber->Capture();
	auto pCloud = DefaultPreprocess(pCloudRaw);
	float sphereSize = EstimateSphereRadius(m_pObject->getPosition(), pCloud);
	m_radiusObject = sphereSize;
	m_radiusColliderContact = sphereSize + thickness_collider_contact;
	m_radiusColliderClick = m_radiusColliderContact + thickness_collider_click;
	std::cout << "Estimation finished. balloon size is:" << RadiusObject() << std::endl;
	return true;
}

float PclHandStateReader::EstimateSphereRadius(const Eigen::Vector3f &center, pcl_util::pcl_ptr pCloud) {	
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ecex;
	ecex.setClusterTolerance(tol_cluster_dist);
	ecex.setMinClusterSize(min_cluster_size_balloon);
	ecex.setMaxClusterSize(max_cluster_size);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	ecex.setSearchMethod(tree);
	ecex.setInputCloud(pCloud);
	std::vector<pcl::PointIndices> cluster_indices;
	ecex.extract(cluster_indices);
	auto itr_largest = std::max_element(
		cluster_indices.begin(),
		cluster_indices.end(),
		[](pcl::PointIndices pi_small, pcl::PointIndices pi_large) {
			return pi_small.indices.size() < pi_large.indices.size();
		}
	);
	std::cout << cluster_indices.size() << " clusters are detected." << std::endl;
	std::cout << "The size of the cluster is: " << (*itr_largest).indices.size() << std::endl;
	pcl_ptr pCloudSphere(new pcl::PointCloud<pcl::PointXYZ>());
	pCloudSphere->points.resize((*itr_largest).indices.size());
	pCloudSphere->width = (*itr_largest).indices.size();
	pCloudSphere->height = 1;
	pCloudSphere->is_dense = true;
	for (int idx_pt = 0; idx_pt < (*itr_largest).indices.size(); idx_pt++) {
		pCloudSphere->points[idx_pt] = pCloud->points[(*itr_largest).indices[idx_pt]];
	}
	auto itr_pt_farthest = std::max_element(pCloudSphere->points.begin(), pCloudSphere->points.end(),
		[&center](pcl::PointXYZ p_near, pcl::PointXYZ p_far) {
			return pcl_util::squareDist(center, p_near) < pcl_util::squareDist(center, p_far);
		}
	);
	auto itr_pt_nearest = std::min_element(pCloudSphere->points.begin(), pCloudSphere->points.end(),
		[&center](pcl::PointXYZ p_near, pcl::PointXYZ p_far) {
			return pcl_util::squareDist(center, p_near) < pcl_util::squareDist(center, p_far);
		}
	);
	std::cout << "posBalloon: " << center.transpose() << std::endl;
	std::cout << "nearest point distance:" << pcl_util::squareDist(center, *itr_pt_nearest);
	std::cout << "farthest point distance:" << pcl_util::squareDist(center, *itr_pt_farthest);
	return sqrtf(pcl_util::squareDist(center, *itr_pt_farthest));
}

bool PclHandStateReader::EstimateHandState(
	dynaman::HandState& state,
	const Eigen::Vector3f& center,
	pcl_util::pcl_ptr pCloud
){
	if (pCloud->empty()) {
		return false;
	}
	pcl::PointXYZ center_pcl(center.x(), center.y(), center.z());
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	kdTree.setInputCloud(pCloud);
	std::vector<int> pointIdxSearch;
	std::vector<float> pointSquareDists;
	kdTree.radiusSearch(center_pcl, RadiusColliderClick(), pointIdxSearch, pointSquareDists);

	auto itr_contact_min = std::find_if(
		pointSquareDists.begin(),
		pointSquareDists.end(),
		[this](float dist)
		{
			return dist > RadiusObject() * RadiusObject();
		}
	);
	auto itr_click_min = std::find_if(
		itr_contact_min,
		pointSquareDists.end(),
		[this](float dist) {
			return dist > RadiusColliderContact() * RadiusColliderContact();
		}
	);
	int num_points_contact = std::distance(itr_contact_min, itr_click_min);
	state = (num_points_contact > thres_contact_num) ? HandState::TOUCH : HandState::NONCONTACT;
	std::cout
		<< "contact points: " << num_points_contact
		<< (state == HandState::TOUCH ? " [Contact]" : " ")
		<< std::endl;
	return true;
}

float PclHandStateReader::RadiusObject() {
	return m_radiusObject;
}

float PclHandStateReader::RadiusColliderContact() {
	return m_radiusColliderContact;
}

float PclHandStateReader::RadiusColliderClick() {
	return m_radiusColliderClick;
}

bool PclHandStateReader::Read(HandState& state) {
	return EstimateHandState(
		state,
		m_pObject->getPosition(),
		DefaultPreprocess(m_pPclGrabber->Capture())
	);
}

