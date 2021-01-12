#include <iostream>
#include <string>
#include <thread>
#include "state_type.hpp"
#include "ActionHandler.hpp"

using namespace dynaman;

int main(int argc, char** argv) {
	auto ah = ActionHandler::create();

	std::string str_free("I am Free.");
	std::string str_touch("I am Touch.");
	std::string str_hold_init("I am Hold Init.");
	std::string str_hold_finger_up("I am Hold Finger Up.");
	std::string str_hold_finger_down("I am Hold Finger Down.");
	ah->setAtFree([&str_free]() {
		std::cout << str_free << "(" << timeGetTime() << ")" << std::endl; 
		}
	);
	ah->setAtTouch([&str_touch]() {
		std::cout << str_touch << "(" << timeGetTime() << ")" << std::endl; 
		}
	);
	ah->setAtHeldInit([&str_hold_init]() {
		std::cout << str_hold_init << "(" << timeGetTime() << ")" << std::endl; 
		}
	);
	ah->setAtHeldFingerDown([&str_hold_finger_down]() {
		std::cout << str_hold_finger_down << "(" << timeGetTime() << ")" << std::endl; 
		}
	);
	ah->setAtHeldFingerUp([&str_hold_finger_up]() {
		std::cout << str_hold_finger_up << "(" << timeGetTime() << ")" << std::endl; 
		}
	);
	ah->setOnHold([]() {std::cout << "Now holded." << std::endl; });
	ah->setOnClick([]() {std::cout << "Now Clicked." << std::endl; });
	ah->setOnRelease([]() {std::cout << "Now Released." << std::endl; });
	int i = 0;
	for (; i < 20; i++) {
		std::cout << i << ": ";
		ah->update(HandState::HOLD_FINGER_DOWN);
		ah->execute();
		std::this_thread::sleep_for(std::chrono::milliseconds(33));
	}
	for (; i < 30; i++) {
		std::cout << i << ": ";
		ah->update(HandState::HOLD_FINGER_UP);
		ah->execute();
		std::this_thread::sleep_for(std::chrono::milliseconds(33));
	}

	for (; i < 45; i++) {
		std::cout << i << ": ";
		ah->update(HandState::HOLD_FINGER_DOWN);
		ah->execute();
		std::this_thread::sleep_for(std::chrono::milliseconds(33));
	}

	return 0;
	
	//
}