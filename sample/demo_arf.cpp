#include "CameraDevice.hpp"
#include "StereoTracker.hpp"
#include "additionalGain.hpp"
#include "haptic_icon.hpp"
#include "geometryUtil.hpp"
#include "manipulator.hpp"
#include <fstream>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {

	std::string target_image_name("blue_target_r50mm.png");
	std::string leftCamId("32434751");
	std::string rightCamId("43435351");
	//Eigen::Vector3f pos_sensor(-125.652f, -871.712f, 13.3176f);
	//Eigen::Quaternionf quo_sensor(0.695684f, -0.718283f, -0.0089647f, 0.00359883f);
	Eigen::Vector3f pos_sensor(-130.83, -874.526, 8.1065);
	Eigen::Quaternionf quo_sensor(0.699819, -0.714301, -0.00510867, -0.00126173);
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
	
	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen())
		return ENXIO;
	haptic_icon::SetGeometry(pAupa);

	auto objPtr = dynaman::FloatingObject::Create(
		Eigen::Vector3f::Zero(),
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		0,//-0.036e-3f,
		50.f);
	std::this_thread::sleep_for(std::chrono::seconds(30));

	auto initTime = timeGetTime();
	Eigen::VectorXi amplitudes(11);
	amplitudes <<
		0, 205, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	float amp_single = 1;
	while (timeGetTime() - initTime < 10000) {
		DWORD observationTime;
		Eigen::Vector3f pos;
		bool tracked = tracker.observe(observationTime, pos, objPtr);
		if (tracked) {
			Eigen::MatrixXf points = pos.replicate(1, 11);
			std::this_thread::sleep_for(std::chrono::seconds(1));
			pAupa->AppendGainSync(autd::DeviceSpecificFocalPointGain::Create(points, amplitudes));
			//pAupa->AppendGainSync(autd::FocalPointGain::Create(pos));
			//pAupa->AppendModulationSync(autd::SineModulation::Create(200, amp_single, 0.5 * amp_single));
			//std::this_thread::sleep_for(std::chrono::seconds(5));
			pAupa->AppendModulationSync(autd::Modulation::Create(255));
			std::this_thread::sleep_for(std::chrono::seconds(5));

		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	pAupa->AppendGainSync(autd::NullGain::Create());
	pAupa->Close();
	stereoCamPtr->close();

	return 0;
}