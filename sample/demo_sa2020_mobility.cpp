#include "CameraDevice.hpp"
#include "StereoTracker.hpp"
#include "additionalGain.hpp"
#include "haptic_icon.hpp"
#include "geometryUtil.hpp"
#include "haptic_icon_strategies/multiplex_strategy.hpp"
#include "odcs.hpp"
#include <fstream>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {

	std::cout << "Enter a log file name: ";
	std::string filename;
	std::getline(std::cin, filename);
	//std::string target_image_name("blue_target_cover.png");
	std::string target_image_name("blue_target_wo_cover.png");
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
	std::cout << "opening cameras..." << std::endl;
	stereoCamPtr->open();
	std::cout << "initializing a tracker..." << std::endl;
	dynaman::stereoTracker tracker(stereoCamPtr, extractorPtrLeft, extractorPtrRight, pos_sensor, quo_sensor, sensor_bias);

	dynaman::odcs manipulator(tracker);
	haptic_icon::Initialize(manipulator);
	Eigen::Vector3f gainP = 10 * Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = 5 * Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = 5 * Eigen::Vector3f::Constant(-0.05f);
	Eigen::Vector3f pos_init(0, 0, 0);
	manipulator.ocsPtr->SetGain(gainP, gainD, gainI);
	auto objPtr = dynaman::FloatingObject::Create(
		pos_init,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		-0.038e-3f,
		50.f);
	std::cout << "workspace " << std::endl
		<< "lower bound: " << objPtr->lowerbound().transpose() << std::endl
		<< "upper bound: " << objPtr->upperbound().transpose() << std::endl;

	int duration = 180000;//1800000; //30min
	int loopPeriod = 10;
	int freq = 100;
	float lambda = 0;
	dynaman::multiplex_strategy strategy(gainP, gainD, gainI, loopPeriod, filename, freq, lambda);

	std::thread th_control([&manipulator, &strategy, &objPtr, &duration]() {
		strategy.run(manipulator, objPtr, duration);
		}
	);
	
	Eigen::Vector3f posCenter(0.f, 0.f, 0.f);
	Eigen::Vector3f posRight(300.f, 0.f, 0.f);
	Eigen::Vector3f posLeft(-300.f, 0.f, 0.f);
	Eigen::Vector3f posHigh(0.f, 0.f, 300.f);
	Eigen::Vector3f posLow(0.f, 0.f, -300.f);
	// params for circular trajectory
	float orbit_radius = 150;
	float orbit_period = 3.6f;
	Eigen::Vector3f posCircleInit(orbit_radius, 0.f, 0.f);
	float timeTrans = 2.0f;
	objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime() / 1000.f, posCenter, posRight));
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime() / 1000.f, posRight, posLeft));
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime() / 1000.f, posLeft, posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime() / 1000.f, posCenter, posHigh));
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime() / 1000.f, posHigh, posLow));
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime() / 1000.f, posLow, posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime() / 1000.f, posCenter, posCircleInit));
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	objPtr->SetTrajectory(dynaman::TrajectoryCircle::Create(posCenter, orbit_radius, pi / 2.f, 0.f, orbit_period, 0.f, timeGetTime() / 1000.f));
	std::this_thread::sleep_for(std::chrono::milliseconds(8000));
	objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime() / 1000.f, objPtr->getPosition(), posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	th_control.join();
	manipulator.Close();

	return 0;
}