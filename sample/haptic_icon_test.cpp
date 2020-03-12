#include "CameraDevice.hpp"
#include "StereoTracker.hpp"
#include "haptic_icon.hpp"
#include "odcs.hpp"

int main(int argc, char** argv) {

	std::string target_image_name("target.png");
	std::string leftCamId("32434751");
	std::string rightCamId("43435351");
	Eigen::Vector3f pos_sensor(0.0f, 0.0f, 0.0f);
	Eigen::Quaternionf quo_sensor(-7.98142e-05f, -0.0105203f, -0.0108641f, 0.707026f);
	int lowerb = 100, upperb = 255, hist_size = 30;
	std::cout << "loading image ..." << std::endl;
	cv::Mat img_target = cv::imread(target_image_name);
	if (img_target.empty()) {
		std::cerr << "Failed to open the target image." << std::endl;
	}
	std::cout << "initializing extractor ..." << std::endl;
	//initialize tracking algorithm
	std::vector<cv::Mat> imgs_target = { img_target };
	auto extractorPtr = imgProc::hue_backproject_extractor::create(imgs_target, lowerb, upperb, hist_size);

	//initialize camera
	std::cout << "initializing camera ..." << std::endl;
	auto leftCamPtr = ximeaCameraDevice::create(leftCamId);
	auto rightCamPtr = ximeaCameraDevice::create(rightCamId);
	auto stereoCamPtr = dynaman::stereoCamera::create(leftCamPtr, rightCamPtr);
	std::cout << "opening camera..." << std::endl;
	stereoCamPtr->open();
	std::cout << "initializing tracker..." << std::endl;
	dynaman::stereoTracker tracker(stereoCamPtr, extractorPtr, pos_sensor, quo_sensor);

	dynaman::odcs manipulator(tracker);
	haptic_icon::Initialize(manipulator);
	auto objPtr = dynaman::FloatingObject::Create(
		Eigen::Vector3f(0, 0, 0),
		Eigen::Vector3f::Constant(-300),
		Eigen::Vector3f::Constant(300),
		1e-5f,
		50.f);

	int active_id = 0;
	Eigen::Vector3f pos_active_autd = manipulator.ocsPtr->CentersAUTD().col(active_id);
	manipulator.StartControl();
	auto timeInit = timeGetTime();
	while (timeGetTime() - timeInit < 30000) {
		float offsetLength = 300;
		Eigen::Vector3f pos = objPtr->getPosition();
		Eigen::Vector3f posTgt = pos + offsetLength * (pos - pos_active_autd).normalized();
		objPtr->updateStatesTarget(posTgt, Eigen::Vector3f(0, 0, 0));
		Sleep(5);
	}
	manipulator.Close();
	return 0;
}