#include <iostream>
#include <memory>
#include <Eigen/Geometry>
#include "autd3.hpp"
#include "CameraDevice.hpp"
#include "StereoTracker.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"

int main(int argc, char** argv) {

	//configuration settings
	std::string filename_prefix;
	std::cout << "Enter filename prefix:";
	std::cin >> filename_prefix;
	std::string filenabe_obs = filename_prefix + "_obs.csv";
	std::string filename_control = filename_prefix + "_control.csv";
	Eigen::Vector3f posInit(0.0f, 0.0f, 0.0f);
	float dist_interval = 50;
	auto staytime = std::chrono::seconds(10);
	std::string target_image_name("blue_target_no_cover.png");

	auto pTracker = haptic_icon::CreateTracker(target_image_name);
	pTracker->open();

	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen()) {
		return ENXIO;
	}
	haptic_icon::SetGeometry(pAupa);

	auto pObject = dynaman::FloatingObject::Create(
		Eigen::Vector3f(0, 0, 0),
		Eigen::Vector3f::Constant(-400),
		Eigen::Vector3f::Constant(400),
		-0.036e-3f,
		50.f
	);

	auto pManipulator = dynaman::MultiplexManipulator::Create(
		20 * Eigen::Vector3f::Constant(-1.6f), // gainP
		5 * Eigen::Vector3f::Constant(-4.0f), // gainD
		1 * Eigen::Vector3f::Constant(-0.05f), //gainI
		100, //freqLM
		10,
		5,
		0
	);
	pManipulator->StartManipulation(pAupa, pTracker, pObject);

	pManipulator->FinishManipulation();
	
}
