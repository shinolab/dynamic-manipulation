#include <memory>
#include <thread>
#include <chrono>
#include "autd3.hpp"
#include "StereoTracker.hpp"
#include "strategy.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"

using namespace dynaman;

int main(int argc, char** argv ) {

	/*user-defined configurations*/
	Eigen::Vector3f pos_init(0, 0, 0);
	Eigen::Vector3f gainP = 20 * Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = 5 * Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = 1 * Eigen::Vector3f::Constant(-0.05f);
	std::string target_image_name("blue_target_no_cover.png");
	Eigen::Vector3f pos_sensor(-125.652f, -871.712f, 13.3176f);
	Eigen::Quaternionf quo_sensor(0.695684f, -0.718283f, -0.0089647f, 0.00359883f);
	std::string leftCamId("32434751");
	std::string rightCamId("43435351");
	/*end of user-defined configurations*/

	auto trackerPtr = stereoTracker::create(
		stereoCamera::create(ximeaCameraDevice::create(leftCamId), ximeaCameraDevice::create(rightCamId)),
		imgProc::hue_backproject_extractor::create(target_image_name),
		imgProc::hue_backproject_extractor::create(target_image_name),
		pos_sensor,
		quo_sensor
	);
	auto aupaPtr = std::make_shared<autd::Controller>();
	haptic_icon::Initialize(aupaPtr);
	auto objPtr = dynaman::FloatingObject::Create(
		pos_init,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		-0.036e-3f,
		50.f
	);
	std::cout << "creating strategy ..." << std::endl;
	auto strategyPtr = MultiplexStrategy::Create(
		gainP,
		gainD,
		gainI
	);
	std::cout << "opening aupa ..." << std::endl;
	aupaPtr->Open(autd::LinkType::ETHERCAT);
	if (!aupaPtr->isOpen())
		return ENXIO;
	std::cout << "opening tracker ..." << std::endl;
	trackerPtr->open();
	std::cout << "executing strategy ... " << std::endl;
	strategyPtr->Initialize(aupaPtr, trackerPtr, objPtr);
	//for (int i = 0; i < 3; i++) {
	//	std::cout << "execute: " << i << std::endl;
	//	strategyPtr->Execute();
	//}
	//return 0;
	//Manipulator manipulator(
	//	aupaPtr,
	//	trackerPtr,
	//	strategyPtr,
	//	objPtr,
	//	10
	//);

	//manipulator.StartControl();

	/*initiate control*/
	DWORD timeInit = timeGetTime();
	strategyPtr->Initialize(aupaPtr, trackerPtr, objPtr);
	if (!trackerPtr->isOpen()) {
		trackerPtr->open();
	}
	if (!aupaPtr->isOpen()) {
		aupaPtr->Open(autd::LinkType::ETHERCAT);
		aupaPtr->AppendGainSync(autd::NullGain::Create());
		aupaPtr->AppendModulationSync(autd::Modulation::Create(255));
	}
	std::thread thControl([&strategyPtr, &timeInit]() 
		{
			unsigned int loopPeriod = 10;
			while (timeGetTime() - timeInit < 120000) {
				{
					DWORD timeLoopInit = timeGetTime();
					strategyPtr->Execute();
					int waitTime = loopPeriod - (timeGetTime() - timeLoopInit);
					timeBeginPeriod(1);
					Sleep(std::max(waitTime, 0));
					timeEndPeriod(1);
				}
			}
		}
	);
	/******************/

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
	objPtr->updateStatesTarget(posLeft);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posLeft));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	objPtr->updateStatesTarget(posRight);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(3.0f, timeGetTime(), posLeft, posRight));
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	objPtr->updateStatesTarget(posCenter);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posRight, posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	objPtr->updateStatesTarget(posHigh);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posHigh));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	objPtr->updateStatesTarget(posLow);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(3.0f, timeGetTime(), posHigh, posLow));
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	objPtr->updateStatesTarget(posCenter);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posLow, posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posCircleInit));
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	objPtr->SetTrajectory(dynaman::TrajectoryCircle::Create(posCenter, orbit_radius, pi / 2.f, 0.f, orbit_period, 0.f, timeGetTime()));
	std::this_thread::sleep_for(std::chrono::milliseconds(6000));
	objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime(), objPtr->getPosition(), posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	//manipulator.Close();
	return 0;
}
