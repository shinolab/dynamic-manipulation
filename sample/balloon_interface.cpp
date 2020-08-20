#include <utility>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "balloon_interface.hpp"

namespace {
	const int thres_contact_num = 50;
	const int thres_touch_num = 5;
	const float thres_hold_num = 10;
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
m_pCloud(new pcl::PointCloud<pcl::PointXYZ>())
{}

std::shared_ptr<balloon_interface> balloon_interface::Create(
	FloatingObjectPtr pObject,
	std::shared_ptr<pcl_grabber> pPointCloudSensor
) {
	return std::make_shared<balloon_interface>(pObject, pPointCloudSensor);
}

void balloon_interface::Open() {

	m_thr_observer = std::thread([this]()
		{
			m_pPclSensor->Open();
			{
				std::lock_guard<std::mutex> lock(m_mtx_is_open);
				m_is_open = true;
			}
			while (IsOpen()) {
				{
					std::lock_guard<std::mutex> lock(m_mtx_pCloud);
					m_pCloud = m_pPclSensor->Capture();
					m_contact_queue.push_back(
						std::make_pair(timeGetTime(), IsContact(m_pObject, m_pCloud))
					);
				}
				m_contact_queue.pop_front();
				auto holdState = DetermineHoldState(m_contact_queue);
				if (IsRunning()) {
					if (holdState == HoldState::HOLD) {
						OnHold();
					}
				}
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
		m_is_open = false;
		if (m_thr_observer.joinable()) {
			m_thr_observer.join();
		}
	}
	m_pPclSensor->Close();
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
	for (auto ic = contact_queue.rbegin() - thres_touch_num; ic != contact_queue.rend(); ic++) {
		if ((*ic).second == true) {
			countContact++;
		}
		else {
			break;
		}
	}
	if (countContact > thres_hold_num) {
		return HoldState::HOLD;
	}
	else if (countContact > thres_touch_num) {
		return HoldState::TOUCH;
	}	
	return HoldState::FREE;
}

bool balloon_interface::IsContact(FloatingObjectPtr pObject, pcl_ptr pCloud) {
	Eigen::Vector3f c = 0.001f * pObject->getPosition();
	pcl::PointXYZ balloon_center(c.x(), c.y(), c.z());
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	std::vector<int> pointIdxSearch;
	std::vector<float> pointSquaredDistances;

	kdtree.radiusSearch(balloon_center, thres_contact_max, pointIdxSearch, pointSquaredDistances);
	auto itr_contact_min = std::find_if(
		pointSquaredDistances.begin(), 
		pointSquaredDistances.end(),
		[this](float dist) 
		{
			return dist > this->thres_contact_min * this->thres_contact_min;
		}
	);
	int num_points_contact = std::distance(pointSquaredDistances.begin(), itr_contact_min);
	return num_points_contact > thres_contact_num;
}

void balloon_interface::OnHold() {
	m_pObject->updateStatesTarget(m_pObject->AveragePosition());
}

balloon_interface::pcl_ptr balloon_interface::CopyPointCloud() {
	return pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>(*m_pCloud));
}

float balloon_interface::ThresContactMin() {
	return thres_contact_min;
};
float balloon_interface::ThresContactMax() {
	return thres_contact_max;
}