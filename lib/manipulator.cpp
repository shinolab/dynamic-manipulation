#include "manipulator.hpp"

namespace dynaman {

	Manipulator::Manipulator(
		std::shared_ptr<autd::Controller> aupa,
		std::shared_ptr<Tracker> tracker,
		std::shared_ptr<Strategy> strategy,
		FloatingObjectPtr objPtr,
		unsigned int loopPeriod)
		:m_aupa(aupa),
		m_tracker(tracker),
		m_objPtr(objPtr),
		m_strategy(strategy),
		m_loopPeriod(loopPeriod),
		m_isRunning(false)
	{}
	
	bool Manipulator::IsRunning() {
		std::shared_lock<std::shared_mutex> lk(m_mtxRunning);
		return m_isRunning;
	}

	void Manipulator::StartControl(autd::LinkType linktype, const std::string& location)
	{
		m_strategy->Initialize(controller(), tracker(), m_objPtr);
		if (!tracker()->isOpen()) {
			tracker()->open();
		}
		if (!controller()->isOpen()) {
			controller()->Open(linktype, location);
			controller()->AppendGainSync(autd::NullGain::Create());
			controller()->AppendModulationSync(autd::Modulation::Create(255));
		}
		{
			std::lock_guard<std::shared_mutex> lock(m_mtxRunning);
			m_isRunning = true;
		}
		m_thControl
			= std::thread([this](){
				while (this->IsRunning()){
					{
						DWORD timeLoopInit = timeGetTime();
						{
							std::lock_guard<std::shared_mutex> lock(this->m_mtxStrategy);
							this->strategy()->Execute();
						}
						int waitTime = m_loopPeriod - (timeGetTime() - timeLoopInit);
						timeBeginPeriod(1);
						Sleep(std::max(waitTime, 0));
						timeEndPeriod(1);
					}
				}
			}
		);
	}

	void Manipulator::PauseControl() {
		{
			std::lock_guard<std::shared_mutex> lk(m_mtxRunning);
			m_isRunning = false;
		}
		if (m_thControl.joinable()) {
			m_thControl.join();
		}
		m_aupa->AppendGainSync(autd::NullGain::Create());
	}

	void Manipulator::Close()
	{
		PauseControl();
		m_aupa->Close();
	}

	std::shared_ptr<autd::Controller> Manipulator::controller() {
		return m_aupa;
	}

	std::shared_ptr<Tracker> Manipulator::tracker() {
		return m_tracker;
	}

	std::shared_ptr<Strategy> Manipulator::strategy() {
		return m_strategy;
	}

	void Manipulator::ApplyStrategy(std::shared_ptr<Strategy> new_strategy) {
		{
			std::lock_guard<std::shared_mutex> lock(m_mtxStrategy);
			m_strategy = new_strategy;
		}
	}
}