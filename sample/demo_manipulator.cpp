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

	Eigen::Vector3f pos_init(0, 0, 0);
	Eigen::Vector3f gainP = 20 * Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = 5 * Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = 1 * Eigen::Vector3f::Constant(-0.05f);

	//std::string target_image_name("blue_target_cover.png");
	std::string target_image_name("blue_target_no_cover.png");
	std::string leftCamId("32434751");
	std::string rightCamId("43435351");
	Eigen::Vector3f pos_sensor(-125.652f, -871.712f, 13.3176f);
	Eigen::Quaternionf quo_sensor(0.695684f, -0.718283f, -0.0089647f, 0.00359883f);
	Eigen::Vector3f sensor_bias(0.0f, 0.0f, 0.0f);
	int lowerb = 10, upperb = 255, hist_size = 30;
	std::cout << "loading target images ..." << std::endl;
	cv::Mat img_target = cv::imread(target_image_name);
	if (img_target.empty()) {
		std::cerr << "Failed to open the target image." << std::endl;
	}
	std::cout << "initializing an extractor ..." << std::endl;
	std::vector<cv::Mat> imgs_target = { img_target };
	auto extractorPtrLeft = imgProc::hue_backproject_extractor::create(imgs_target, lowerb, upperb, hist_size);
	auto extractorPtrRight = imgProc::hue_backproject_extractor::create(imgs_target, lowerb, upperb, hist_size);

	std::cout << "initializing a camera ..." << std::endl;
	auto leftCamPtr = ximeaCameraDevice::create(leftCamId);
	auto rightCamPtr = ximeaCameraDevice::create(rightCamId);
	auto stereoCamPtr = dynaman::stereoCamera::create(leftCamPtr, rightCamPtr);
	//stereoCamPtr->open();
	std::cout << "initializing a tracker..." << std::endl;
	auto trackerPtr = std::make_shared<stereoTracker>(stereoCamPtr, extractorPtrLeft, extractorPtrRight, pos_sensor, quo_sensor, sensor_bias);
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
		gainI,
		100
	);
	std::cout << "opening aupa ..." << std::endl;
	aupaPtr->Open(autd::LinkType::ETHERCAT);
	if (!aupaPtr->isOpen())
		return ENXIO;
	std::cout << "opening tracker ..." << std::endl;
	trackerPtr->open();
	std::cout << "executing strategy ... " << std::endl;

	/*initiate control*/

	strategyPtr->Initialize(aupaPtr, trackerPtr, objPtr);
	if (!trackerPtr->isOpen()) {
		trackerPtr->open();
	}
	if (!aupaPtr->isOpen()) {
		aupaPtr->Open(autd::LinkType::ETHERCAT);
		aupaPtr->AppendGainSync(autd::NullGain::Create());
		aupaPtr->AppendModulationSync(autd::Modulation::Create(255));
	}
	std::thread thControl([&strategyPtr]() 
		{
			DWORD timeInit = timeGetTime();
			while (timeGetTime() - timeInit < 90000) {
				{
					strategyPtr->Execute();
					//std::cout << "elapsed: " << timeGetTime() - loopInit << std::endl;
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
	thControl.join();
	aupaPtr->Close();
	return 0;
}
