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
		0,
		std::make_shared<arfModelFocusSphereExp50mm>()
	);
	manipulator.m_pAupa = pAupa;
	manipulator.m_pObject = pObject;
	manipulator.m_pTracker = pTracker;

	Eigen::Vector3f forceTarget(2, 2, 0);
	auto duty = manipulator.ComputeDuty(forceTarget, pos);

	Eigen::MatrixXf posRel = pos.replicate(1, pAupa->geometry()->numDevices()) - CentersAutd(pAupa->geometry());
	std::cout << CentersAutd(pAupa->geometry()) << std::endl;
	Eigen::Vector3f forceResult = manipulator.arfModel()->arf(posRel, RotsAutd(pAupa->geometry()))* duty;

	std::cout << "Duty: " << duty.transpose() << std::endl;
	std::cout << "Sum (Duty): " << duty.sum() << std::endl;
	std::cout << "ForceTarget: " << forceTarget.transpose() << std::endl;
	std::cout << "ForceResult: " << forceResult.transpose() << std::endl;
	std::cout << "Force Matrix:" << std::endl << manipulator.arfModel()->arf(posRel, RotsAutd(pAupa->geometry())) << std::endl;;

	auto sequence = manipulator.CreateDriveSequence(duty, pos);
	for (auto itr = sequence.begin(); itr != sequence.end(); itr++) {
		std::cout << itr->second << ",";
	}
	std::cout << std::endl;
	//for (auto itr = sequence.begin(); itr != sequence.end(); itr++) {
	//	manipulator.mux.AddOrder(
	//		[&itr]() { std::cout << itr->second; },
	//		500000
	//	);
	//}
	for (int i = 0; i < sequence.size(); i++) {
		manipulator.mux.AddOrder(
			[&sequence, i]() {std::cout << i << ": " << sequence[i].second << std::endl; },
			500000
		);
	}
	manipulator.mux.Start();
	std::this_thread::sleep_for(std::chrono::seconds(3));
	manipulator.mux.Stop();

	return 0;
}