#include <memory>
#include <thread>
#include <chrono>
#include "autd3.hpp"
#include "StereoTracker.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"

using namespace dynaman;

int main(int argc, char** argv ) {

	/*user-defined configurations*/
	Eigen::Vector3f pos_init(0, 0, 0);
	std::string target_image_name("blue_target_no_cover.png");
	Eigen::Vector3f pos_sensor(-125.652f, -871.712f, 13.3176f);
	Eigen::Quaternionf quo_sensor(0.695684f, -0.718283f, -0.0089647f, 0.00359883f);
	std::string leftCamId("32434751");
	std::string rightCamId("43435351");
	/*end of user-defined configurations*/

	auto pAupa = std::make_shared<autd::Controller>();

	auto pObject = dynaman::FloatingObject::Create(
		pos_init,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		-0.036e-3f,
		50.f
	);

	std::cout << "opening aupa ..." << std::endl;
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen())
		return ENXIO;
	haptic_icon::SetGeometry(pAupa);
	std::cout << "opening tracker ..." << std::endl;
	auto pTracker = stereoTracker::create(
		stereoCamera::create(ximeaCameraDevice::create(leftCamId), ximeaCameraDevice::create(rightCamId)),
		imgProc::hue_backproject_extractor::create(target_image_name),
		imgProc::hue_backproject_extractor::create(target_image_name),
		pos_sensor,
		quo_sensor
	);
	pTracker->open();

	/*initiate control*/
	std::cout << "creating strategy ..." << std::endl;
	auto pManipulator = MultiplexManipulator::Create(
		20 * Eigen::Vector3f::Constant(-1.6f), // gainP
		5 * Eigen::Vector3f::Constant(-4.0f), // gainD
		1 * Eigen::Vector3f::Constant(-0.05f), //gainI
		100 //freqLM
	);
	pManipulator->StartManipulation(pAupa, pTracker, pObject);

	Eigen::Vector3f posCenter(0.f, 0.f, 0.f);
	Eigen::Vector3f posRight(300.f, 0.f, 0.f);
	Eigen::Vector3f posLeft(-300.f, 0.f, 0.f);
	Eigen::Vector3f posHigh(0.f, 0.f, 300.f);
	Eigen::Vector3f posLow(0.f, 0.f, -250.f);
	// params for circular trajectory
	float orbit_radius = 150;
	float orbit_period = 3.f;
	Eigen::Vector3f posCircleInit(orbit_radius, 0.f, 0.f);
	std::this_thread::sleep_for(std::chrono::seconds(5));
	//traslation maneuver:
	pObject->updateStatesTarget(posLeft);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posLeft));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	pObject->updateStatesTarget(posRight);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(3.0f, timeGetTime(), posLeft, posRight));
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	pObject->updateStatesTarget(posCenter);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posRight, posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	pObject->updateStatesTarget(posHigh);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posHigh));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	pObject->updateStatesTarget(posLow);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(3.0f, timeGetTime(), posHigh, posLow));
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	pObject->updateStatesTarget(posCenter);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posLow, posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posCircleInit));
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	pObject->SetTrajectory(dynaman::TrajectoryCircle::Create(posCenter, orbit_radius, pi / 2.f, 0.f, orbit_period, 0.f, timeGetTime()));
	std::this_thread::sleep_for(std::chrono::milliseconds(6000));
	pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime(), pObject->getPosition(), posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));

	pManipulator->FinishManipulation();
	pAupa->Close();
	return 0;
}
