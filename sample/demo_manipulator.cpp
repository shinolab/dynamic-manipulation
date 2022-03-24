#include <memory>
#include <thread>
#include <chrono>
#include "autd3.hpp"
#include "StereoTracker.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"

using namespace dynaman;

int main(int argc, char** argv ) {

	std::string target_image_name("blue_target_r50mm.png");
	auto pTracker = haptic_icon::CreateTracker(target_image_name);
	pTracker->open();
	if (!pTracker->isOpen())
		return ENXIO;

	auto pAupa = haptic_icon::CreateController();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen())
		return ENXIO;

	auto pManipulator = MultiplexManipulator::Create(pAupa, pTracker);

	auto pObject = dynaman::FloatingObject::Create(
		Eigen::Vector3f::Zero(), //initial position
		Eigen::Vector3f::Constant(-600), // lower bound of workspace
		Eigen::Vector3f::Constant(600), // upper bound of workspace
		50.0f // radius [mm]
	);

	std::cout << "Starting Manipulaion ... " << std::endl;
	pManipulator->StartManipulation(pObject);
	std::this_thread::sleep_for(std::chrono::seconds(8)); // wait for stabilization

	//params for straight maneuvers
	Eigen::Vector3f posCenter(0.f, 0.f, 0.f);
	Eigen::Vector3f posRight(280.f, 0.f, 0.f);
	Eigen::Vector3f posLeft(-280.f, 0.f, 0.f);
	Eigen::Vector3f posHigh(0.f, 0.f, 300.f);
	Eigen::Vector3f posLow(0.f, 0.f, -200.f);
	// params for circular trajectory
	float orbit_radius = 200;
	float orbit_period = 3.f;
	Eigen::Vector3f posCircleInit(orbit_radius, 0.f, 0.f);

	//Maneuver without open-loop control
	pObject->updateStatesTarget(posLeft);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	pObject->updateStatesTarget(posRight);
	std::this_thread::sleep_for(std::chrono::milliseconds(1250));
	pObject->updateStatesTarget(posCenter);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	pObject->updateStatesTarget(posHigh);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	pObject->updateStatesTarget(posLow);
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	pObject->updateStatesTarget(posCenter);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	//Maneuvers with open-loop control
	pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posCircleInit));
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	pObject->SetTrajectory(dynaman::TrajectoryCircle::Create(posCenter, orbit_radius, pi / 2.f, 0.f, orbit_period, 0.f, timeGetTime()));
	std::this_thread::sleep_for(std::chrono::milliseconds(6100));
	pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime(), pObject->getPosition(), posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	
	pObject->SetTrajectory(dynaman::TrajectoryInfShape::Create(
		posCenter,
		300,
		400,
		5,
		timeGetTime()
	));
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime(), pObject->getPosition(), posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	pObject->updateStatesTarget(posCenter);

	getchar();
	pManipulator->FinishManipulation();
	pAupa->Close();
	return 0;
}
