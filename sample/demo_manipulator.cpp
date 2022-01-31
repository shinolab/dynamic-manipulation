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
	std::string target_image_name("blue_target_r50mm.png");
	float balloonRadias = 50.0f;
	float balloonWeight = 0.0f;
	/*end of user-defined configurations*/

	auto pObject = dynaman::FloatingObject::Create(
		pos_init,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		balloonWeight,
		balloonRadias
	);

	auto pTracker = haptic_icon::CreateTracker(target_image_name);
	pTracker->open();

	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen())
		return ENXIO;
	haptic_icon::SetGeometry(pAupa);

	auto pManipulator = MultiplexManipulator::Create(
		20 * Eigen::Vector3f::Constant(-1.6f), // gainP
		5 * Eigen::Vector3f::Constant(-4.0f), // gainD
		1 * Eigen::Vector3f::Constant(-0.05f) //gainI
	);

	std::cout << "Starting Manipulaion ... " << std::endl;
	pManipulator->StartManipulation(pAupa, pTracker, pObject);
	std::this_thread::sleep_for(std::chrono::seconds(8)); // wait for stabilization

	Eigen::Vector3f posCenter(0.f, 0.f, 0.f);
	Eigen::Vector3f posRight(280.f, 0.f, 0.f);
	Eigen::Vector3f posLeft(-280.f, 0.f, 0.f);
	Eigen::Vector3f posHigh(0.f, 0.f, 300.f);
	Eigen::Vector3f posLow(0.f, 0.f, -200.f);
	// params for circular trajectory
	float orbit_radius = 200;
	float orbit_period = 3.f;
	Eigen::Vector3f posCircleInit(orbit_radius, 0.f, 0.f);

	//traslation maneuver without open-loop control
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

	//Translation maneuvers with open-loop control
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
