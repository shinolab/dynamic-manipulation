#include "CameraDevice.hpp"
#include "StereoTracker.hpp"
#include "additionalGain.hpp"
#include "haptic_icon.hpp"
#include "geometryUtil.hpp"
#include "haptic_icon_strategies/multiplex_strategy.hpp"
#include "haptic_icon_strategies/simple_strategy.hpp"
#include "odcs.hpp"
#include <fstream>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {

	std::string target_image_name("blue_target_no_cover.png");
	std::string leftCamId("32434751");
	std::string rightCamId("43435351");
	Eigen::Vector3f pos_sensor(-125.652f, -871.712f, 13.3176f);
	Eigen::Quaternionf quo_sensor(0.695684f, -0.718283f, -0.0089647f, 0.00359883f);
	Eigen::Vector3f sensor_bias(0.0f, 0.0f, 0.0f);
	int lowerb = 10, upperb = 255, hist_size = 30;
	std::cout << "initializing an extractor ..." << std::endl;
	std::cout << "loading target images ..." << std::endl;
	cv::Mat img_target = cv::imread(target_image_name);
	if (img_target.empty()) {
		std::cerr << "Failed to open the target image." << std::endl;
	}
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

	auto objPtr = dynaman::FloatingObject::Create(
		Eigen::Vector3f::Zero(),
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		-0.036e-3f,
		50.f);

	auto initTime = timeGetTime();
	Eigen::VectorXi amplitudes(11);
	amplitudes <<
		0, 0, 0, 0, 155, 0, 0, 0, 0, 0, 0;
	while (timeGetTime() - initTime < 5000) {
		DWORD observationTime;
		Eigen::Vector3f pos;
		bool tracked = tracker.observe(observationTime, pos, objPtr);
		if (tracked) {
			Eigen::MatrixXf points = pos.replicate(1, 11);
			manipulator.ocsPtr->_autd.AppendGainSync(autd::DeviceSpecificFocalPointGain::Create(points, amplitudes));
			manipulator.ocsPtr->_autd.AppendModulationSync(autd::Modulation::Create(150));
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	manipulator.Close();

	return 0;
}