#ifndef _DYNAMAN_BALLOON_INTERFACE_HPP
#define _DYNAMAN_BALLOON_INTERFACE_HPP

#include <thread>
#include <functional>
#include <queue>
#include <memory>
#include <mutex>
#include <Eigen/Geometry>
#include "FloatingObject.hpp"
#include "pcl_grabber.hpp"
#include <librealsense2/rs.hpp>
#include <Windows.h>
#pragma comment (lib, "winmm")

namespace dynaman {

	class balloon_interface {
	public:
		using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
		enum class REF_FRAME { GLOBAL, USER };
		enum class HoldState { FREE, TOUCH, HOLD };
		balloon_interface(
			FloatingObjectPtr pObject,
			std::shared_ptr<pcl_grabber> pPointCloudSensor
		);
		~balloon_interface() = default;

		static std::shared_ptr<balloon_interface> Create(
			FloatingObjectPtr pObject,
			std::shared_ptr<pcl_grabber> pPointCloudSensor
		);
		void Open();
		void Run();
		void Pause();
		void Close();
		bool IsOpen();
		bool IsRunning();
		bool IsContact(FloatingObjectPtr pObject, pcl_ptr pCloud);
		HoldState DetermineHoldState(std::deque<std::pair<DWORD, bool>> contact_queue);
		void OnHold();
		pcl_ptr CopyPointCloud();
		float DetermineBalloonSize(pcl_ptr pCloud, const Eigen::Vector3f& posBalloon);
		pcl_ptr TrimCloudOutsideWorkspace(FloatingObjectPtr pObject, pcl_ptr pCloud);
		float RadiusColliderMin();
		float RadiusColliderMax();

	private:
		void InitializeCollider();
		const int size_queue = 10;
		REF_FRAME m_ref_frame = REF_FRAME::GLOBAL;
		std::deque<std::pair<DWORD, bool>> m_contact_queue;
		FloatingObjectPtr m_pObject;
		bool m_is_running;
		bool m_is_open;
		pcl_ptr m_pCloud;
		float m_thres_contact_min;
		float m_thres_contact_max;
		std::shared_ptr<pcl_grabber> m_pPclSensor;
		std::mutex m_mtx_is_open;
		std::mutex m_mtx_is_running;
		std::mutex m_mtx_pCloud;
		std::mutex m_mtx_collider;
		std::thread m_thr_observer;
	};
}

#endif // !_DYNAMAN_BALLOON_INTERFACE_HPP