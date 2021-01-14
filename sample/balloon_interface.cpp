#include <algorithm>
#include <utility>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "manipulator.hpp"
#include "pcl_util.hpp"
#include "HandStateReader.hpp"
#include "ActionHandler.hpp"
#include "balloon_interface.hpp"


using namespace dynaman;

balloon_interface::balloon_interface(
	FloatingObjectPtr pObject,
	std::shared_ptr<Manipulator> pManipulator,
	std::shared_ptr<PclHandStateReader> pHandStateReader,
	std::shared_ptr<ActionHandler> pActionHandler
):m_is_running(false),
m_is_open(false),
m_contact_queue(thres_hold_num, std::make_pair(0, false)),
m_pHandStateReader(pHandStateReader),
m_pObject(pObject)
{}

std::shared_ptr<balloon_interface> balloon_interface::Create(
	FloatingObjectPtr pObject,
	std::shared_ptr<PclHandStateReader> pHandStateReader
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


