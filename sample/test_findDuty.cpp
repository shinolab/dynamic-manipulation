#include <fstream>
#include <iostream>
#include "autd3.hpp"
#include "FloatingObject.hpp"
#include "odcs.hpp"
#include "strategy.hpp"
#include "StereoTracker.hpp"
#include "haptic_icon.hpp"
#include "GainPlan.hpp"

float castfunc(float value) {
	return value;
}

int main(int argc, char** argv) {

	float freqf = 100;
	int freqi = 100;
	unsigned int frequi = 100;
	std::cout << "float: " << castfunc(freqf) << ", int: " << castfunc(freqi) << ", unsigned int: " << castfunc(frequi) << std::endl;
	return 0;
	auto pAupa = std::make_shared<autd::Controller>();
	haptic_icon::SetGeometry(pAupa);

	std::string target_image_name("target.png");
	Eigen::Vector3f pos_sensor(-125.652f, -871.712f, 13.3176f);
	Eigen::Quaternionf quo_sensor(0.695684f, -0.718283f, -0.0089647f, 0.00359883f);
	std::string leftCamId("32434751");
	std::string rightCamId("43435351");
	/*end of user-defined configurations*/

	std::cout << "creating tracker ptr" << std::endl;
	auto pTracker = dynaman::stereoTracker::create(
		dynaman::stereoCamera::create(photoDevice::create(target_image_name), photoDevice::create(target_image_name)),
		imgProc::hue_backproject_extractor::create(target_image_name),
		imgProc::hue_backproject_extractor::create(target_image_name),
		pos_sensor,
		quo_sensor
	);

	auto tracker = dynaman::stereoTracker(
		dynaman::stereoCamera::create(photoDevice::create(target_image_name), photoDevice::create(target_image_name)),
		imgProc::hue_backproject_extractor::create(target_image_name),
		imgProc::hue_backproject_extractor::create(target_image_name),
		pos_sensor,
		quo_sensor
	);
	std::cout << "creating manipulator" << std::endl;
	dynaman::odcs manipulator(tracker);
	manipulator.Initialize();

	haptic_icon::SetGeometry(manipulator);
	std::cout << "creating object" << std::endl;
	auto objPtr = dynaman::FloatingObject::Create(
		Eigen::Vector3f::Zero(),
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600)
	);
	auto gainP = Eigen::Vector3f::Constant(20);
	auto gainD = Eigen::Vector3f::Constant(5);
	auto gainI = Eigen::Vector3f::Constant(0.01);
	std::cout << "creating strategy" << std::endl;
	auto pStrategy
		= std::dynamic_pointer_cast<dynaman::MultiplexStrategy>(
			dynaman::MultiplexStrategy::Create(
				gainP,
				gainD,
				gainI
			)
		);
	std::cout << "initializing strategy" << std::endl;
	pStrategy->Initialize(pAupa, pTracker, objPtr);

	std::ofstream ofs("20200703_findDutyTest.csv");
	ofs << "#, fxTgt, fyTgt, fzTgt, fres_x(old), fres_y(old), fres_z(old), fres_x(new), fres_y(new), fres_z(new),"
		<< "u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, u10" << std::endl;
	std::cout << "CentersDevices(new):\n" << dynaman::CentersAutd(pAupa->geometry()) << std::endl;
	std::cout << "CentersDevices(old):\n" << manipulator.Controller()->CentersAUTD() << std::endl;

	std::cout << "DirectionsDevices(new):\n" << dynaman::DirectionsAutd(pAupa->geometry()) << std::endl;
	std::cout << "DirectionsDevices(old):\n" << manipulator.Controller()->DirectionsAUTD() << std::endl;
	for (int i = 0; i < 100; i++) {
		Eigen::Vector3f posNew = objPtr->getPosition() + 10 * Eigen::Vector3f::Random();
		objPtr->updateStates(timeGetTime(), posNew);
		Eigen::Vector3f accel
			= gainP.asDiagonal() * (objPtr->getPosition() - objPtr->getPositionTarget())
			+ gainD.asDiagonal() * (objPtr->averageVelocity() - objPtr->getVelocityTarget())
			+ gainI.asDiagonal() * objPtr->getIntegral()
			+ objPtr->getAccelTarget();
		Eigen::Vector3f force
			= objPtr->totalMass() * accel
			+ objPtr->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665e3f);
		auto dutiesOld = manipulator.Controller()->FindDutyQpMultiplex(force, objPtr->getPosition(), 0.0f).transpose();
		auto dutiesNew = pStrategy->ComputeDuty(force, objPtr->getPosition()).transpose();
		auto dutiesDIff = (dutiesOld - dutiesNew).norm();
		Eigen::MatrixXf posRel = objPtr->getPosition().replicate(1, pAupa->geometry()->numDevices()) -  dynaman::CentersAutd(pAupa->geometry());
		Eigen::MatrixXf forceMatOld = manipulator.Controller()->arfModelPtr->arf(posRel, manipulator.Controller()->eulerAnglesAUTD);
		Eigen::MatrixXf forceMatNew = pStrategy->arfModel()->arf(posRel, dynaman::RotsAutd(pAupa->geometry()));
		Eigen::Vector3f forceResultOld = forceMatOld * dutiesOld;
		Eigen::Vector3f forceResultNew = forceMatNew * dutiesNew;
		Eigen::MatrixXf forceMatDiff = forceMatNew - forceMatOld;
		std::cout //<< "forceMatDiff, " << forceMatDiff.cwiseAbs().sum()
			<< ", dutyDiff, " << dutiesDIff
			<< ", forceDiff(new): " << (forceResultNew - force).norm()
			<< ", forceDiff(old): " << (forceResultOld - force).norm()
			<< std::endl;
		ofs << i << ", " << force.x() << ", " << force.y() << ", " << force.z() << ", "
			<< forceResultOld.x() << ", " << forceResultOld.y() << ", " << forceResultOld.z() << ", "
			<< forceResultNew.x() << ", " << forceResultNew.y() << ", " << forceResultNew.z() << ", "
			<< dutiesOld(0) << ", " << dutiesOld(1) << ", " << dutiesOld(2) << ", "
			<< dutiesOld(3) << ", " << dutiesOld(4) << ", " << dutiesOld(5) << ", "
			<< dutiesOld(6) << ", " << dutiesOld(7) << ", " << dutiesOld(8) << ", "
			<< dutiesOld(9) << ", " << dutiesOld(10) << ", " 
			<< dutiesNew(0) << ", " << dutiesNew(1) << ", " << dutiesNew(2) << ", "
			<< dutiesNew(3) << ", "	<< dutiesNew(4) << ", " << dutiesNew(5) << ", "
			<< dutiesNew(6) << ", " << dutiesNew(7) << ", " << dutiesNew(8) << ", "
			<< dutiesNew(9) << ", " << dutiesNew(10) << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	ofs.close();
	return 0;
}