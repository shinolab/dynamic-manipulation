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
	Eigen::Vector3f pos_init(0, -50, 0);
	std::string target_image_name("blue_target_no_cover.png");
	/*end of user-defined configurations*/

	auto pObject = dynaman::FloatingObject::Create(
		pos_init,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		-0.036e-3f,
		50.f
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
		1 * Eigen::Vector3f::Constant(-0.05f), //gainI
		100, //freqLM
		10,
		5,
		0
	);
	pManipulator->StartManipulation(pAupa, pTracker, pObject);

	Eigen::Vector3f posCenter(0.f, -50.f, 0.f);
	Eigen::Vector3f posRight(300.f, -50.f, 0.f);
	Eigen::Vector3f posLeft(-300.f, -50.f, 0.f);
	Eigen::Vector3f posHigh(0.f, -50.f, 300.f);
	Eigen::Vector3f posLow(0.f, -50.f, -250.f);
	// params for circular trajectory
	float orbit_radius = 150;
	float orbit_period = 3.f;
	Eigen::Vector3f posCircleInit(orbit_radius, 0.f, 0.f);
	std::this_thread::sleep_for(std::chrono::seconds(5));
	//traslation maneuver:
	//pObject->updateStatesTarget(posLeft);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posLeft));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//pObject->updateStatesTarget(posRight);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(3.0f, timeGetTime(), posLeft, posRight));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1250));
	//pObject->updateStatesTarget(posCenter);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posRight, posCenter));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//pObject->updateStatesTarget(posHigh);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posHigh));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//pObject->updateStatesTarget(posLow);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(3.0f, timeGetTime(), posHigh, posLow));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	//pObject->updateStatesTarget(posCenter);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posLow, posCenter));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posCircleInit));
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	pObject->SetTrajectory(dynaman::TrajectoryCircle::Create(posCenter, orbit_radius, pi / 2.f, 0.f, orbit_period, 0.f, timeGetTime()));
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime(), pObject->getPosition(), posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	getchar();
	pManipulator->FinishManipulation();
	pAupa->Close();
	return 0;
}
