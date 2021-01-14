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

		balloon_interface(
			FloatingObjectPtr pObject,
			std::shared_ptr<autd::Controller> pAupa,
			std::shared_ptr<Manipulator> pManipulator,
			std::shared_ptr<PclHandStateReader> pHandStateReader
		);

		~balloon_interface() = default;

		static std::shared_ptr<balloon_interface> Create(
			FloatingObjectPtr pObject,
			std::shared_ptr<autd::Controller> pAupa,
			std::shared_ptr<Manipulator> pManipulator,
			std::shared_ptr<PclHandStateReader> pHandStateReader
		);
		void Open();
		void Run();
		void Pause();
		void Close();
		bool IsOpen();
		bool IsRunning();


	private:
		const int size_queue = 10;
		std::deque<std::pair<DWORD, bool>> m_contact_queue;
		std::shared_ptr<PclHandStateReader> m_pHandStateReader;
		std::shared_ptr<Manipulator> m_pManipulator;
		std::shared_ptr<ActionHandler> m_pActionHandler;
		FloatingObjectPtr m_pObject;
		bool m_is_running;
		bool m_is_open;
		std::mutex m_mtx_is_open;
		std::mutex m_mtx_is_running;
		std::thread m_thr_observer;
		
	};
}

#endif // !_DYNAMAN_BALLOON_INTERFACE_HPP