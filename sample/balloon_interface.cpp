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
#include "pcl_util.hpp"
#include "HandStateReader.hpp"
#include "balloon_interface.hpp"

namespace {
	const int thres_hold_num = 10;
	const int thres_touch_num = 1;
}

using namespace dynaman;

balloon_interface::balloon_interface(
	FloatingObjectPtr pObject,
	std::shared_ptr<HandStateReader> pHandStateReader
):m_is_running(false),
m_is_open(false),
m_contact_queue(thres_hold_num, std::make_pair(0, false)),
m_pHandStateReader(pHandStateReader),
m_pObject(pObject)
{}

std::shared_ptr<balloon_interface> balloon_interface::Create(
	FloatingObjectPtr pObject,
	std::shared_ptr<HandStateReader> pHandStateReader
) {
	return std::make_shared<balloon_interface>(pObject, pHandStateReader);
}

void balloon_interface::Open() {

	m_thr_observer = std::thread([this]()
		{
			{
				std::lock_guard<std::mutex> lock(m_mtx_is_open);
				m_is_open = true;
			}
			m_pHandStateReader->Initialize();
			
			while (IsOpen()) 
			{
				HandState handState;
				bool isValidState = m_pHandStateReader->Read(handState);
				if (isValidState) {
					auto is_contact = handState != HandState::NONCONTACT;
					m_contact_queue.push_back(
						std::make_pair(timeGetTime(), is_contact)
					);
					m_contact_queue.pop_front();
					auto holdState = DetermineHoldState(m_contact_queue);
					if (holdState == HoldState::HOLD && IsRunning()) {
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


void balloon_interface::OnHold() {
	std::cout << "HOLDING.";
	auto currentPositionGlobal = m_pObject->AveragePosition();
	m_pObject->updateStatesTarget(currentPositionGlobal);
}

