#include <algorithm>
#include <iostream>
#include "GainPlan.hpp"
#include "haptic_icon.hpp"

using namespace dynaman;


int main(int argc, char** argv) {

	auto pAupa = std::make_shared<autd::Controller>();
	haptic_icon::SetGeometry(pAupa);
	auto pTracker = haptic_icon::CreateTracker("blue_target_r50mm.png");

	MultiplexManipulator::Create(
		pAupa,
		pTracker
	);

	return 0;
}