#include <utility>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl_util.hpp"
#include "balloon_interface.hpp"

namespace {
	const int thres_contact_num = 150;
	const int thres_touch_num = 5;
	const int thres_hold_num = 10;
}

using namespace dynaman;


balloon_interface::balloon_interface(
	FloatingObjectPtr pObject,
	std::shared_ptr<pcl_grabber> pPointCloudSensor
):m_pObject(pObject),
m_is_running(false),
m_is_open(false),
m_pPclSensor(pPointCloudSensor),
m_contact_queue(thres_hold_num, std::make_pair(0, false)),
m_pCloud(new pcl::PointCloud<pcl::PointXYZ>()),
m_thres_contact_min(0.07f),
m_thres_contact_max(0.1f)
{}

std::shared_ptr<balloon_interface> balloon_interface::Create(
	FloatingObjectPtr pObject,
	std::shared_ptr<pcl_grabber> pPointCloudSensor
) {
	return std::make_shared<balloon_interface>(pObject, pPointCloudSensor);
}

void balloon_interface::InitializeCollider()
{	
	std::lock_guard<std::mutex> lock(m_mtx_pCloud);
	std::cout << "Configuring collider ...";
	auto pCloudInit = m_pPclSensor->Capture();
	auto pCloudInside = pcl_util::TrimPointsOutsideWorkspace(m_pObject, pCloudInit);
	pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
	vg_filter.setInputCloud(pCloudInside);
	vg_filter.setLeafSize(0.005f, 0.005f, 0.005f);
	pcl_ptr pCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>());
	vg_filter.filter(*pCloudFiltered);
	float balloonSize = DetermineBalloonSize(pCloudFiltered, 0.001f * m_pObject->AveragePosition());
	{
		std::lock_guard<std::mutex> lock(m_mtx_collider);
		m_thres_contact_min = balloonSize;
		m_thres_contact_max = balloonSize + 0.03f;
	}
	std::cout << "Complete. balloon size is:" << RadiusColliderMin() << std::endl;
}

void balloon_interface::Open() {

	m_thr_observer = std::thread([this]()
		{
			m_pPclSensor->Open();
			{
				std::lock_guard<std::mutex> lock(m_mtx_is_open);
				m_is_open = true;
			}
			InitializeCollider();
			
			while (IsOpen()) 
			{
				{
					std::lock_guard<std::mutex> lock(m_mtx_pCloud);
					auto pCloudRaw = m_pPclSensor->Capture();
					auto pCloudFiltered = pcl_util::TrimPointsOutsideWorkspace(m_pObject, pCloudRaw);
					pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
					vg_filter.setInputCloud(pCloudFiltered);
					vg_filter.setLeafSize(0.005f, 0.005f, 0.005f);
					vg_filter.filter(*m_pCloud);
				}
				auto is_contact = IsContact(m_pObject, m_pCloud);
				m_contact_queue.push_back(
					std::make_pair(timeGetTime(), is_contact)
				);
				m_contact_queue.pop_front();
				auto holdState = DetermineHoldState(m_contact_queue);
				if (IsRunning()) {
					if (holdState == HoldState::HOLD) {
						OnHold();
					}
				}
				std::this_thread::sleep_for(std::chrono::microseconds(30));
			}
		}
	);
}

void balloon_interface::Pause() {
	std::lock_guard<std::mutex> lock(m_mtx_is_running);
	m_is_running = false;
}

void balloon_interface::Run() {
	std::lock_guard<std::mutex> lock(m_mtx_is_running);
	m_is_running = true;
}

void balloon_interface::Close() {
	{
		std::lock_guard<std::mutex> lock(m_mtx_is_running);
		m_is_running = false;
	}
	{
		std::lock_guard<std::mutex> lock(m_mtx_is_open);
		m_is_open = false;
	}
	if (m_thr_observer.joinable()) {
		m_thr_observer.join();
	}
}

bool balloon_interface::IsOpen() {
	std::lock_guard<std::mutex> lock(m_mtx_is_open);
	return m_is_open;
}

bool balloon_interface::IsRunning() {
	std::lock_guard<std::mutex> lock(m_mtx_is_running);
	return m_is_running;
}

balloon_interface::HoldState balloon_interface::DetermineHoldState(
	std::deque<std::pair<DWORD, bool>> contact_queue
) {
	int countContact = 0;
	for (auto ic = contact_queue.rbegin(); ic != contact_queue.rend(); ic++) {
		if ((*ic).second == true) {
			countContact++;
		}
		else {
			break;
		}
	}
	if (countContact >= thres_hold_num) {
		return HoldState::HOLD;
	}
	else if (countContact >= thres_touch_num) {
		return HoldState::TOUCH;
	}	
	return HoldState::FREE;
}

bool balloon_interface::IsContact(FloatingObjectPtr pObject, pcl_ptr pCloud) {
	if (pCloud->empty()) {
		return false;
	}
	Eigen::Vector3f c = 0.001f * pObject->getPosition();
	pcl::PointXYZ balloon_center(c.x(), c.y(), c.z());

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pCloud);
	std::vector<int> pointIdxSearch;
	std::vector<float> pointSquaredDistances;
	kdtree.radiusSearch(balloon_center, RadiusColliderMax(), pointIdxSearch, pointSquaredDistances);

	auto itr_contact_min = std::find_if(
		pointSquaredDistances.begin(), 
		pointSquaredDistances.end(),
		[this](float dist) 
		{
			return dist > this->RadiusColliderMin() * this->RadiusColliderMin();
		}
	);
	int num_points_contact = std::distance(itr_contact_min, pointSquaredDistances.end());
	auto is_contact = (num_points_contact > thres_contact_num);
	std::cout
		<< "contact points: " << num_points_contact
		<< (is_contact ? " [Contact]" : " ")
		<< std::endl;
	return is_contact;
}

void balloon_interface::OnHold() {
	std::cout << "HOLDING.";
	auto currentPositionGlobal = m_pObject->AveragePosition();
	m_pObject->updateStatesTarget(currentPositionGlobal);
}

balloon_interface::pcl_ptr balloon_interface::CopyPointCloud() {
	std::lock_guard<std::mutex> lock(m_mtx_pCloud);
	return pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>(*m_pCloud));
}

float balloon_interface::DetermineBalloonSize(
	balloon_interface::pcl_ptr pCloud,
	const Eigen::Vector3f& posBalloon
) {
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ecex;
	ecex.setClusterTolerance(0.05f);
	ecex.setMinClusterSize(20);
	ecex.setMaxClusterSize(10000);
	ecex.setSearchMethod(tree);
	ecex.setInputCloud(pCloud);
	ecex.extract(cluster_indices);
	auto itr_largest = std::max_element(
		cluster_indices.begin(), 
		cluster_indices.end(),
		[](pcl::PointIndices pi1, pcl::PointIndices pi2) {
			return pi1.indices.size() < pi2.indices.size();
		}
	);
	std::cout << cluster_indices.size() << " clusters are identified." << std::endl;
	std::cout << "The size of the cluster is: " << (*itr_largest).indices.size() << std::endl;
	pcl_ptr pCloudBalloon(new pcl::PointCloud<pcl::PointXYZ>());
	pCloudBalloon->points.resize((*itr_largest).indices.size());
	pCloudBalloon->width = (*itr_largest).indices.size();
	pCloudBalloon->height = 1;
	pCloudBalloon->is_dense = true;
	for (int idx_pt = 0; idx_pt < (*itr_largest).indices.size(); idx_pt++) {
		pCloudBalloon->points[idx_pt] = pCloud->points[(*itr_largest).indices[idx_pt]];
	}
	m_pCloud = pCloudBalloon;
	auto itr_pt_farthest = std::max_element(pCloudBalloon->points.begin(), pCloudBalloon->points.end(),
		[&posBalloon](pcl::PointXYZ p_far, pcl::PointXYZ p_near) {
			return pcl_util::squareDist(posBalloon, p_far) < pcl_util::squareDist(posBalloon, p_near);
		}
	);
	auto itr_pt_nearest = std::min_element(pCloudBalloon->points.begin(), pCloudBalloon->points.end(),
		[&posBalloon](pcl::PointXYZ p_far, pcl::PointXYZ p_near) {
			return pcl_util::squareDist(posBalloon, p_far) < pcl_util::squareDist(posBalloon, p_near);
		}
	);
	std::cout << "posBalloon: " << posBalloon.transpose() << std::endl;
	std::cout << "nearest point distance:" << pcl_util::squareDist(posBalloon, *itr_pt_nearest);
	std::cout << "farthest point distance:" << pcl_util::squareDist(posBalloon, *itr_pt_farthest);
	return sqrtf(pcl_util::squareDist(posBalloon, *itr_pt_farthest));
}

float balloon_interface::RadiusColliderMin() {
	std::lock_guard<std::mutex> lock(m_mtx_collider);
	return m_thres_contact_min;
}

float balloon_interface::RadiusColliderMax() {
	std::lock_guard<std::mutex> lock(m_mtx_collider);
	return m_thres_contact_max;
}
