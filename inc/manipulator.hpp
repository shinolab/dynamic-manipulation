#ifndef _DYNAMAN_MANIPULATOR_HPP
#define _DYNAMAN_MANIPULATOR_HPP

#include <memory>
#include "autd3.hpp"
#include "FloatingObject.hpp"
#include "tracker.hpp"
#include "strategy.hpp"

namespace dynaman{
		class Manipulator {
		std::shared_ptr<autd::Controller> m_aupa;
		std::shared_ptr<Tracker> m_tracker;
		FloatingObjectPtr m_objPtr;
		std::shared_ptr<Strategy> m_strategy;
		unsigned int m_loopPeriod;
	private:
		std::thread m_thControl;
		std::shared_mutex m_mtxStrategy;
		std::shared_mutex m_mtxRunning;
		bool m_isRunning;

	public:
		Manipulator(
			std::shared_ptr<autd::Controller> aupaPtr,
			std::shared_ptr<Tracker> tracker,
			std::shared_ptr<Strategy> strategy,
			FloatingObjectPtr objPtr,
			unsigned int loopPeriod
		);
		void StartControl(autd::LinkType linktype = autd::LinkType::ETHERCAT, const std::string& location = "");
		void PauseControl();
		void Close();
		void ApplyStrategy(std::shared_ptr<Strategy> new_strategy);
		bool IsRunning();

		std::shared_ptr<autd::Controller> controller();
		std::shared_ptr<Tracker> tracker();
		std::shared_ptr<Strategy> strategy();
	};
}
#endif // !_DYNAMAN_MANIPULATOR_HPP