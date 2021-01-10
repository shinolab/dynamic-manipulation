#ifndef _DYNAMAN_BALLOON_INTERFACE_HPP
#define _DYNAMAN_BALLOON_INTERFACE_HPP

#include <thread>
#include <functional>
#include <queue>
#include <memory>
#include <mutex>
#include <Eigen/Geometry>
#include <librealsense2/rs.hpp>
#include "FloatingObject.hpp"
#include "HandStateReader.hpp"
#include "pcl_grabber.hpp"
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
			std::shared_ptr<HandStateReader> pHandStateReader
		);
		~balloon_interface() = default;

		static std::shared_ptr<balloon_interface> Create(
			FloatingObjectPtr pObject,
			std::shared_ptr<HandStateReader> pHandStateReader
		);
		void Open();
		void Run();
		void Pause();
		void Close();
		bool IsOpen();
		bool IsRunning();
		HoldState DetermineHoldState(std::deque<std::pair<DWORD, bool>> contact_queue);
		void OnHold();


	private:
		const int size_queue = 10;
		REF_FRAME m_ref_frame = REF_FRAME::GLOBAL;
		std::deque<std::pair<DWORD, bool>> m_contact_queue;
		std::shared_ptr<HandStateReader> m_pHandStateReader;
		FloatingObjectPtr m_pObject;
		bool m_is_running;
		bool m_is_open;
		std::mutex m_mtx_is_open;
		std::mutex m_mtx_is_running;
		std::thread m_thr_observer;
	};
}

#endif // !_DYNAMAN_BALLOON_INTERFACE_HPP