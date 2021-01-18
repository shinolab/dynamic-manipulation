#include <memory>
#include <utility>
#include "state_type.hpp"
#include "ActionHandler.hpp"

#pragma comment (lib, "winmm")

namespace {
	const int hand_state_queue_size = 10;
	const int thres_hold_num = 7;
	const int thres_touch_num = 3;
	const int thres_hold_time_ms = 300;
	const int thres_release_num = 2;
	const int thres_finger_down_num = 3;
	const int thres_finger_up_num = 3;
	const int thres_click_time_ms = 10000;
}

using namespace dynaman;

ActionHandler::ActionHandler()
:m_contactCount(0),
m_noncontactCount(0),
m_fingerUpCount(0),
m_fingerDownCount(0),
m_currentState(HoldState::FREE),
m_previousState(HoldState::FREE),
m_timeStateChange(timeGetTime()),
m_atFree([]() {}),
m_atTouch([]() {}),
m_atHeldInit([]() {}),
m_atHeldFingerDown([]() {}),
m_atHeldFingerUp([]() {}),
m_onTouch([]() {}),
m_onHold([]() {}),
m_onRelease([]() {}),
m_onClick([]() {}) {}

std::shared_ptr<ActionHandler> ActionHandler::create() {
	return std::make_shared<ActionHandler>();
}

void ActionHandler::update(
	HandState newHandState
) {
	switch (m_currentState) {
	case HoldState::FREE:
		updateAtFree(newHandState);
		break;
	case HoldState::TOUCH:
		updateAtTouch(newHandState);
		break;
	case HoldState::HELD_INIT:
		updateAtHeldInit(newHandState);
		break;
	case HoldState::HELD_FINGER_DOWN:
		updateAtHeldFingerDown(newHandState);
		break;
	case HoldState::HELD_FINGER_UP:
		updateAtHeldFingerUp(newHandState);
		break;
	default:
		break;
	}
}

void ActionHandler::execute() {
	switch (m_currentState)
	{
	case dynaman::HoldState::FREE:
		m_atFree();
		break;
	case dynaman::HoldState::TOUCH:
		m_atTouch();
		break;
	case dynaman::HoldState::HELD_INIT:
		m_atHeldInit();
		break;
	case dynaman::HoldState::HELD_FINGER_UP:
		m_atHeldFingerUp();
		break;
	case dynaman::HoldState::HELD_FINGER_DOWN:
		m_atHeldFingerDown();
		break;
	default:
		break;
	}
}

void ActionHandler::updateAtFree(
	HandState newHandState
) {
	if (newHandState == HandState::NONCONTACT) {
		return;
	}
	m_contactCount++;
	if (m_contactCount > thres_touch_num) {
		changeStateTo(HoldState::TOUCH);
		return;
	}
}

void ActionHandler::updateAtTouch(
	HandState newHandState
) {
	if (newHandState == HandState::NONCONTACT) {
		changeStateTo(HoldState::FREE);
		m_onTouch();
		return;
	}
	m_contactCount++;
	if (m_contactCount >= thres_hold_num && timeGetTime() - m_timeStateChange > thres_hold_time_ms) {
		changeStateTo(HoldState::HELD_INIT);
		m_onHold();
		return;
	}
}

void ActionHandler::updateAtHeldInit(
	HandState newHandState
) {
	switch (newHandState)
	{
	case dynaman::HandState::NONCONTACT:
		m_noncontactCount++;
		if (m_noncontactCount >= thres_release_num) {
			changeStateTo(HoldState::FREE);
			m_onRelease();
		}
		break;
	case dynaman::HandState::HOLD_FINGER_UP:
		m_fingerUpCount++;
		m_fingerDownCount = 0;
		if (m_fingerUpCount > thres_finger_up_num) {
			changeStateTo(HoldState::HELD_FINGER_UP);
		}
		break;
	case dynaman::HandState::HOLD_FINGER_DOWN:
		m_fingerUpCount = 0;
		m_fingerDownCount++;
		if (m_fingerDownCount > thres_finger_down_num) {
			changeStateTo(HoldState::HELD_FINGER_DOWN);
		}
		break;
	default:
		break;
	}
}

void ActionHandler::updateAtHeldFingerDown(
	HandState newHandState
) {
	switch (newHandState)
	{
	case dynaman::HandState::NONCONTACT:
		m_noncontactCount++;
		if (m_noncontactCount >= thres_release_num) {
			changeStateTo(HoldState::FREE);
			m_onRelease();
		}
		break;
	case dynaman::HandState::HOLD_FINGER_UP:
		m_noncontactCount = 0;
		m_fingerUpCount++;
		m_fingerDownCount = 0;
		if (m_fingerUpCount > thres_finger_up_num) {
			changeStateTo(HoldState::HELD_FINGER_UP);
		}
		break;
	case dynaman::HandState::HOLD_FINGER_DOWN:
		m_noncontactCount = 0;
		m_fingerUpCount = 0;
		break;
	default:
		break;
	}
}

void ActionHandler::updateAtHeldFingerUp(
	HandState newHandState
) {
	switch (newHandState)
	{
	case dynaman::HandState::NONCONTACT:
		m_noncontactCount++;
		if (m_noncontactCount >= thres_release_num) {
			changeStateTo(HoldState::FREE);
			m_onRelease();
		}
		break;
	case dynaman::HandState::HOLD_FINGER_UP:
		m_noncontactCount = 0;
		m_fingerDownCount = 0;
		m_fingerUpCount++;
		break;
	case dynaman::HandState::HOLD_FINGER_DOWN:
		m_noncontactCount = 0;
		m_fingerUpCount = 0;
		m_fingerDownCount++;
		if (m_fingerDownCount > thres_finger_up_num) {
			if (m_previousState == HoldState::HELD_FINGER_DOWN
				&& timeGetTime() - m_timeStateChange < thres_click_time_ms)
			{
				m_onClick();
			}
			changeStateTo(HoldState::HELD_FINGER_DOWN);
		}
		break;
	default:
		break;
	}
}

void ActionHandler::changeStateTo(
	HoldState newHoldState
) {
	m_previousState = m_currentState;
	m_currentState = newHoldState;
	m_contactCount = 0;
	m_noncontactCount = 0;
	m_fingerDownCount = 0;
	m_fingerUpCount = 0;
	m_timeStateChange = timeGetTime();
}

void ActionHandler::setAtFree(
	std::function<void()> handler
) {
	m_atFree = handler;
}

void ActionHandler::setAtTouch(std::function<void()> handler) {
	m_atTouch = handler;
}

void ActionHandler::setAtHeldInit(std::function<void()> handler) {
	m_atHeldInit = handler;
}

void ActionHandler::setAtHeldFingerDown(std::function<void()> handler) {
	m_atHeldFingerDown = handler;
}

void ActionHandler::setAtHeldFingerUp(std::function<void()> handler) {
	m_atHeldFingerUp = handler;
}

void ActionHandler::setOnTouch(std::function<void()> handler) {
	m_onTouch = handler;
}

void ActionHandler::setOnHold(std::function<void()> handler) {
	m_onHold = handler;
}

void ActionHandler::setOnRelease(std::function<void()> handler) {
	m_onRelease = handler;
}

void ActionHandler::setOnClick(std::function<void()> handler) {
	m_onClick = handler;
}

HoldState ActionHandler::state() {
	return m_currentState;
}