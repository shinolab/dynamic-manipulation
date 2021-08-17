#include "iostream"
#include "manipulator.hpp"
#include "autd3.hpp"
#include "haptic_icon.hpp"
#include "WinMultiplexer.hpp"
#include "GainPlan.hpp"

using namespace dynaman;

int main(int argc, char** argv) {

	/*user-defined configurations*/
	Eigen::Vector3f pos(0, 0, 0);
	std::string target_image_name("blue_target_no_cover.png");
	/*end of user-defined configurations*/

	auto pObject = dynaman::FloatingObject::Create(
		pos,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		0,
		50.f
	);

	auto pTracker = haptic_icon::CreateTracker(target_image_name);
	
	auto pAupa = std::make_shared<autd::Controller>();

	haptic_icon::SetGeometry(pAupa);
	Eigen::Vector3f gainP(1, 1, 1);
	Eigen::Vector3f gainD(1, 1, 1);
	Eigen::Vector3f gainI(0, 0, 0);

	VarMultiplexManipulator manipulator(
		gainP,
		gainD,
		gainI,
		10000,
		33,
		10,
		1.0,
		0,
		std::make_shared<arfModelFocusSphereExp50mm>()
	);
	manipulator.m_pAupa = pAupa;
	manipulator.m_pObject = pObject;
	manipulator.m_pTracker = pTracker;

	Eigen::Vector3f forceTarget(0,0,3);
	std::cout << "computing duty ... " << std::endl;
	auto duty = manipulator.ComputeDuty(forceTarget, pos);
	auto duty_copy = duty;
	for (int i = 0; i < duty.size(); i++) {
		std::cout << duty[i] << ", ";
	}
	std::cout << std::endl;
	std::cout << "computing sequence (new)" << std::endl;
	auto sequence = manipulator.CreateDriveSequence(duty, pos);
	std::cout << "computing sequence (old)" << std::endl;

	auto sequence_old = manipulator.CreateDriveSequenceOld(duty_copy, pos);
	std::cout << "duration (new): " << std::endl;
	for (auto itr = sequence.begin(); itr != sequence.end(); itr++) {
		std::cout << itr->second << ",";
	}
	std::cout << std::endl;
	std::cout << "duration (old): " << std::endl;
	for (auto itr = sequence_old.begin(); itr != sequence_old.end(); itr++) {
		std::cout << itr->second << ",";
	}
	std::cout << std::endl;
	return 0;
}