#include <algorithm>
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
#include "state_type.hpp"
#include "HandStateReader.hpp"

namespace {
	const float thickness_collider_contact = 0.03;
	const float thickness_collider_click = 0.025;
	const float tol_cluster_dist = 0.02f;
	const int min_cluster_size_balloon = 20;
	const int min_cluster_size_finger = 5;
	const int max_cluster_size = 10000;
	const int thres_contact_num = 100;
	const float thres_click_z = 0.1;
}

using namespace dynaman;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

PclHandStateReader::PclHandStateReader(float radius)
	:m_radiusObject(radius),
	m_radiusColliderContact(radius + thickness_collider_contact),
	m_radiusColliderClick(radius + thickness_collider_contact + thickness_collider_click)
{}

PclHandStateReader::~PclHandStateReader() {};

std::shared_ptr<PclHandStateReader> PclHandStateReader::Create(float radius) {
	return std::make_shared<PclHandStateReader>(radius);
}

pcl_util::pcl_ptr PclHandStateReader::DefaultPreprocess(pcl_util::pcl_ptr pCloud, FloatingObjectPtr pObject) {
	auto pCloudInside = pcl_util::TrimPointsOutsideWorkspace(pObject, pCloud);
	pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
	vg_filter.setInputCloud(pCloudInside);
	vg_filter.setLeafSize(0.005f, 0.005f, 0.005f);
	pcl_ptr pCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>());
	vg_filter.filter(*pCloudFiltered);
	return pCloudFiltered;
}

bool PclHandStateReader::EstimateSphereRadius(pcl_util::pcl_ptr pCloud, const Eigen::Vector3f& center) {

	auto cluster_indices = pcl_util::EuclidianClusterExtraction(pCloud, tol_cluster_dist, min_cluster_size_balloon, max_cluster_size);
	if (cluster_indices.empty()) {
		return false;
	}
	auto itr_largest = std::max_element(
		cluster_indices.begin(),
		cluster_indices.end(),
		[](pcl::PointIndices pi_small, pcl::PointIndices pi_large) {
			return pi_small.indices.size() < pi_large.indices.size();
		}
	);
	std::cout << cluster_indices.size() << " clusters are detected." << std::endl;
	std::cout << "The size of the cluster is: " << (*itr_largest).indices.size() << std::endl;
	auto pCloudSphere = pcl_util::ExtractPointCloud(pCloud, *itr_largest);
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
	std::cout << "nearest point distance:" << pcl_util::squareDist( center, *itr_pt_nearest);
	std::cout << "furthest point distance:" << pcl_util::squareDist(center, *itr_pt_farthest);
	float radiusEstimated = sqrtf(pcl_util::squareDist(center, *itr_pt_farthest));
	m_radiusObject = radiusEstimated;
	m_radiusColliderContact = m_radiusObject + thickness_collider_contact;
	m_radiusColliderClick = m_radiusColliderContact + thickness_collider_click;
	return true;
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
	//std::cout << "num_contact: " << num_points_contact;
	if (num_points_contact < thres_contact_num) {
		//std::cout << std::endl;
		state = HandState::NONCONTACT;
		return true;
	}
	//extract click_collider:
	auto itr_idx_z_max = std::max_element(
		pointIdxSearch.begin() + std::distance(pointSquareDists.begin(), itr_contact_min),
		pointIdxSearch.end(),
		[&pCloud](int idx_small, int idx_large) {
			return pCloud->points[idx_small].z < pCloud->points[idx_large].z;
		}
	);
	auto z_diff = pCloud->points[(*itr_idx_z_max)].z - center.z();
	//std::cout << "z diff: " << z_diff << std::endl;
	if (z_diff > thres_click_z) {
		state = HandState::HOLD_FINGER_UP;
		return true;
	}
	else {
		state = HandState::HOLD_FINGER_DOWN;
		return true;
	}

	int num_points_click = std::distance(itr_click_min, pointSquareDists.end());
	auto pCloudClick = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pCloudClick->points.resize(num_points_click);
	pCloudClick->width = num_points_click;
	pCloudClick->height = 1;
	pCloudClick->is_dense = true;
	for (auto itr = itr_click_min; itr != pointSquareDists.end(); itr++) {
		auto idx_all = std::distance(pointSquareDists.begin(), itr);
		auto idx_click = std::distance(itr_click_min, itr);
		pCloudClick->points[idx_click] = pCloud->points[pointIdxSearch[idx_all]];
	}
	
	auto clusterIndicesClickCollider = pcl_util::EuclidianClusterExtraction(
		pCloudClick,
		tol_cluster_dist,
		min_cluster_size_finger,
		max_cluster_size
	);
	int num_clusters_click = clusterIndicesClickCollider.size();
	state = (num_clusters_click >= 2 ? HandState::HOLD_FINGER_UP : HandState::HOLD_FINGER_DOWN);
	//std::cout << ", num_click: " << num_points_click << ", num_clusters_click : " << num_clusters_click << std::endl;
	return true;
}

pcl_util::pcl_ptr PclHandStateReader::ExtractPointsInsideColliderClick(pcl_util::pcl_ptr pCloud, const Eigen::Vector3f& center) {
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

	//extract click_collider:
	int num_points_click = std::distance(itr_click_min, pointSquareDists.end());
	auto pCloudClick = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pCloudClick->points.resize(num_points_click);
	pCloudClick->width = num_points_click;
	pCloudClick->height = 1;
	pCloudClick->is_dense = true;
	for (auto itr = itr_click_min; itr != pointSquareDists.end(); itr++) {
		auto idx_all = std::distance(pointSquareDists.begin(), itr);
		auto idx_click = std::distance(itr_click_min, itr);
		pCloudClick->points[idx_click] = pCloud->points[pointIdxSearch[idx_all]];
	}

	return pCloudClick;
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

bool PclHandStateReader::Read(HandState& state, pcl_ptr pCloud, const Eigen::Vector3f& center) {
	return EstimateHandState(
		state,
		center,
		pCloud
	);
}

