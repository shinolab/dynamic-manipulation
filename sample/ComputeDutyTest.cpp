#include <iostream>
#include "manipulator.hpp"
#include "autd3.hpp"
#include "haptic_icon.hpp"
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

	Eigen::Vector3f forceTarget(0, 0, 1);
	auto duty = manipulator.ComputeDuty(forceTarget, pos);

	Eigen::MatrixXf posRel = pos.replicate(1, pAupa->geometry()->numDevices()) - CentersAutd(pAupa->geometry());
	//std::cout << CentersAutd(pAupa->geometry()) << std::endl;
	Eigen::Vector3f forceResult = manipulator.arfModel()->arf(posRel, RotsAutd(pAupa->geometry())) * duty;

	std::cout << "Duty: " << duty.transpose() << std::endl;
	std::cout << "Sum (Duty): " << duty.sum() << std::endl;
	std::cout << "ForceTarget: [" << forceTarget.transpose() << "]T mN" << std::endl;
	std::cout << "ForceResult: [" << forceResult.transpose() << "]T mN" << std::endl;
	std::cout << "Error (L2): " << (forceResult - forceTarget).norm() << " mN" << std::endl;
	std::cout << "Force Matrix:" << std::endl << manipulator.arfModel()->arf(posRel, RotsAutd(pAupa->geometry())) << std::endl;;

	return 0;
}