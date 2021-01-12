#ifndef _DYNAMAN_ACTION_HANDLER_HPP
#define _DYNAMAN_ACTION_HANDLER_HPP

#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <Windows.h>
#include "state_type.hpp"

namespace dynaman {

	class ActionHandler{
	public:
		ActionHandler();
		static std::shared_ptr<ActionHandler> create();
		void update(HandState newHandState);
		void execute();
		HoldState state();
		void setAtFree(std::function<void()>);
		void setAtTouch(std::function<void()>);
		void setAtHeldInit(std::function<void()>);
		void setAtHeldFingerDown(std::function<void()>);
		void setAtHeldFingerUp(std::function<void()>);
		void setOnTouch(std::function<void()>);
		void setOnHold(std::function<void()>);
		void setOnRelease(std::function<void()>);
		void setOnClick(std::function<void()>);

	private:
		int m_contactCount;
		int m_noncontactCount;
		int m_fingerUpCount;
		int m_fingerDownCount;
		HoldState m_currentState;
		HoldState m_previousState;
		DWORD m_timeStateChange;
		//executed in every loop depending on the state.
		std::function<void()> m_atFree;
		std::function<void()> m_atTouch;
		std::function<void()> m_atHeldInit;
		std::function<void()> m_atHeldFingerUp;
		std::function<void()> m_atHeldFingerDown;
		//event handlers
		std::function<void()> m_onTouch;
		std::function<void()> m_onHold;
		std::function<void()> m_onRelease;
		std::function<void()> m_onClick;

		void changeStateTo(HoldState newHoldState);
		void updateAtFree(HandState newHandState);
		void updateAtTouch(HandState newHandState);
		void updateAtHeldInit(HandState newHandState);
		void updateAtHeldFingerDown(HandState newHandState);
		void updateAtHeldFingerUp(HandState newHandState);
	};
}

#endif // !_DYNAMAN_ACTION_HANDLER_HPP
