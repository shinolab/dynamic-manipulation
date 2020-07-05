#include "recorder.hpp"
#include <Windows.h>

#pragma comment (lib, "winmm")

namespace dynaman {
	Recorder::Recorder() :m_isRunning(false) {}

	bool Recorder::IsRunning() {
		std::lock_guard<std::mutex> lock(m_mtxRunning);
		return m_isRunning;
	}

	void Recorder::Start(
		FloatingObjectPtr pObject,
		const std::string& filename,
		int queryInterval_ms
	) {
		m_thrLog = std::thread([this, pObject, filename, queryInterval_ms]()
			{
				{
					std::lock_guard<std::mutex> lock(this->m_mtxRunning);
					this->m_isRunning = true;
				}
				this->m_streamLog.open(filename);
				while (this->IsRunning()) {
					DWORD queryTime = timeGetTime();
					Eigen::Vector3f pos, vel, integ;
					pObject->getStates(pos, vel, integ);
					auto posTgt = pObject->getPositionTarget(queryTime);
					auto velTgt = pObject->getVelocityTarget(queryTime);
					auto accelTgt = pObject->getAccelTarget(queryTime);
					m_streamLog << queryTime
						<< "," << pos.x() << "," << pos.y() << "," << pos.z()
						<< "," << vel.x() << "," << vel.y() << "," << vel.z()
						<< "," << integ.x() << "," << integ.y() << "," << integ.z()
						<< "," << posTgt.x() << "," << posTgt.y() << "," << posTgt.z()
						<< "," << velTgt.x() << "," << velTgt.y() << "," << velTgt.z()
						<< "," << accelTgt.x() << "," << accelTgt.y() << "," << accelTgt.z()
						<< std::endl;
					int waitTime = queryInterval_ms - (timeGetTime() - queryTime);
					Sleep(std::max(waitTime, 0));
				}
				m_streamLog.close();
			}
		);
	}

	void Recorder::Stop() {
		{
			std::lock_guard<std::mutex> lock(m_mtxRunning);
			m_isRunning = false;
		}
		if (m_thrLog.joinable()) {
			m_thrLog.join();
		}
	}
}