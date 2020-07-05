#ifndef _DYNAMAN_RECORDER_HPP
#define _DYNAMAN_RECORDER_HPP

#include <string>
#include <fstream>
#include <thread>
#include <mutex>
#include "FloatingObject.hpp"

namespace dynaman {
	class Recorder {
	private:
		bool m_isRunning;

		std::ofstream m_streamLog;
		std::thread m_thrLog;
		std::mutex m_mtxRunning;
	public:
		Recorder();
		void Start(FloatingObjectPtr pObject, const std::string& filename, int queryInterval_ms);
		void Stop();
		bool IsRunning();
	};
}

#endif // !_DYNAMAN_RECORDER_HPP
