#include "ImgProcUtil.hpp"
#include "StereoTracker.hpp"
#include "CameraDevice.hpp"
#include "odcs.hpp"
#include "additionalGain.hpp"
#include "autd3.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#define NOMINMAX
#include <Windows.h>

#pragma comment(lib, "winmm")

namespace {
	const float pi = 3.14159265359f;
	const int num_trans_in_device = 249;
	const float radius_ws = 528.f;
	const float dia_trans = 10.16f;
}

namespace{
	int CustomAddDevice(autd::GeometryPtr geometry, const float theta, const float phi, bool isUpper) {

		Eigen::Vector3f centerLocal(dia_trans * 8.5f, dia_trans * 6.5f, 0.f);
		Eigen::Vector3f euler_angles;
		isUpper ? euler_angles << phi, theta - pi, pi / 2 : euler_angles << phi, theta - pi, -pi / 2;
		Eigen::Vector3f centerGlobal = radius_ws* Eigen::Vector3f(sinf(theta) * cosf(phi), sinf(theta) * sinf(phi), cosf(theta));
		Eigen::Vector3f pos_autd = centerGlobal
			- Eigen::AngleAxisf(euler_angles.x(), Eigen::Vector3f::UnitZ())
			* Eigen::AngleAxisf(euler_angles.y(), Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(euler_angles.z(), Eigen::Vector3f::UnitZ()) * centerLocal;

		std::cout << "AUTD # " << geometry->numDevices() << std::endl
			<< "position: " << pos_autd.transpose() << std::endl
			<< "euler_angles(ZYZ): " << euler_angles.transpose() / pi *180<< std::endl;
		return geometry->AddDevice(pos_autd, euler_angles);

	}

	Eigen::Vector3f Polar2Euler(const float theta, const float phi, bool isUpper) {
		return (isUpper ? Eigen::Vector3f(phi, theta - pi, pi / 2.f) : Eigen::Vector3f(phi, theta - pi, -pi / 2.f));
	}

	Eigen::Vector3f Polar2Position(const float theta, const float phi, bool isUpper) {
		Eigen::Vector3f euler_angles = Polar2Euler(theta, phi, isUpper);
		Eigen::Vector3f centerLocal(dia_trans * 8.5f, dia_trans * 6.5f, 0.f);
		Eigen::Vector3f centerGlobal = radius_ws * Eigen::Vector3f(sinf(theta) * cosf(phi), sinf(theta) * sinf(phi), cosf(theta));
		return centerGlobal
			- Eigen::AngleAxisf(euler_angles.x(), Eigen::Vector3f::UnitZ())
			* Eigen::AngleAxisf(euler_angles.y(), Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(euler_angles.z(), Eigen::Vector3f::UnitZ()) * centerLocal;
	}
}

int main(int argc, char** argv) {
	//user-defined parameters
	Eigen::Vector3f posTgt(0, 0, 0);
	std::string record_name("hapt_log.csv");
	std::string target_image_name("target.png");
	std::string leftCamId("32434751");
	std::string rightCamId("43435351");
	Eigen::Vector3f pos_sensor(0.0f, 0.0f, 0.0f);
	Eigen::Quaternionf quo_sensor(-7.98142e-05f, -0.0105203f, -0.0108641f, 0.707026f);
	int lowerb = 100, upperb = 255, hist_size = 30;
	cv::Mat img_target = cv::imread(target_image_name);

	//initialize tracking algorithm
	std::vector<cv::Mat> imgs_target = { img_target };
	auto extractorPtr = imgProc::hue_backproject_extractor::create(imgs_target, lowerb, upperb, hist_size);

	//initialize camera
	auto leftCamPtr = ximeaCameraDevice::create(leftCamId);
	auto rightCamPtr = ximeaCameraDevice::create(rightCamId);
	auto stereoCamPtr = dynaman::stereoCamera::create(leftCamPtr, rightCamPtr);
	stereoCamPtr->open();
	dynaman::stereoTracker tracker(stereoCamPtr, extractorPtr, pos_sensor, quo_sensor);

	std::vector<std::tuple<float, float, bool>> polar_angles;
	//top autd
	const float theta0 = 0.f, phi0 = -3 * pi / 4; bool isUpper0 = true;
	//upper autd
	const float theta1 = pi / 4, phi1 = pi / 4; bool isUpper1 = true;
	const float theta2 = pi / 4, phi2 = 3 * pi / 4; bool isUpper2 = true;
	const float theta3 = pi / 4, phi3 = - 3 * pi / 4; bool isUpper3 = true;
	const float theta4 = pi / 4, phi4 = - pi / 4; bool isUpper4 = true;
	//middle autd
	const float theta5 = pi / 2, phi5 = pi / 2; bool isUpper5 = true;
	const float theta6 = pi / 2, phi6 = 3 * pi / 2; bool isUpper6 = true;
	//lower autd
	const float theta7 = 3 * pi / 4, phi7 = pi / 4; bool isUpper7 = false;
	const float theta8 = 3 * pi / 4, phi8 = 3 * pi / 4; bool isUpper8 = false;
	const float theta9 = 3 * pi / 4, phi9 = -3 * pi / 4; bool isUpper9 = false;
	const float theta10 = 3 * pi / 4, phi10 = - pi / 4; bool isUpper10 = false;

	dynaman::odcs manipulator(tracker);

	manipulator.AddDevice(Polar2Position(theta9, phi9, isUpper9), Polar2Euler(theta9, phi9, isUpper9));
	manipulator.AddDevice(Eigen::Vector3f(-528.f, 10.16f * 6.5f, -10.16f * 8.5f), Eigen::Vector3f(0, pi / 2, pi));
	manipulator.AddDevice(Polar2Position(theta8, phi8, isUpper8), Polar2Euler(theta8, phi8, isUpper8));
	manipulator.AddDevice(Polar2Position(theta7, phi7, isUpper7), Polar2Euler(theta7, phi7, isUpper7));
	manipulator.AddDevice(Eigen::Vector3f(528.f, -10.16f * 6.5f, -10.16f * 8.5f), Eigen::Vector3f(0, -pi / 2, 0));
	manipulator.AddDevice(Polar2Position(theta10, phi10, isUpper10), Polar2Euler(theta10, phi10, isUpper10));
	manipulator.AddDevice(Polar2Position(theta4, phi4, isUpper4), Polar2Euler(theta4, phi4, isUpper4));
	manipulator.AddDevice(Polar2Position(theta1, phi1, isUpper1), Polar2Euler(theta1, phi1, isUpper1));
	manipulator.AddDevice(Polar2Position(theta2, phi2, isUpper2), Polar2Euler(theta2, phi2, isUpper2));
	manipulator.AddDevice(Polar2Position(theta3, phi3, isUpper3), Polar2Euler(theta3, phi3, isUpper3));
	manipulator.AddDevice(Polar2Position(theta0, phi0, isUpper0), Polar2Euler(theta0, phi0, isUpper0));

	dynaman::FloatingObjectPtr objPtr = dynaman::FloatingObject::Create(posTgt, Eigen::Vector3f(-300, -300, -300), Eigen::Vector3f(300, 300, 300), 1.0e-5, 50);
	manipulator.RegisterObject(objPtr);
	std::cout << "opening manipulator ..." << std::endl;
	manipulator.StartControl();
	auto timeInit = timeGetTime();
	
	std::ofstream ofs(record_name);

	while (timeGetTime() - timeInit < 30000) {
		auto currentTime = timeGetTime();
		auto pos = objPtr->getPosition();
		auto vel = objPtr->getVelocity();
		auto posTgt = objPtr->getPositionTarget();
		auto velTgt = objPtr->getVelocityTarget();
		auto accelTgt = objPtr->getAccelTarget();
		ofs << currentTime << ", " << pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
			<< vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
			<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", "
			<< velTgt.x() << ", " << velTgt.y() << ", " << velTgt.z() << std::endl;
	}
	ofs.close();
	manipulator.Close();
	std::cout << "Press any key to close " << std::endl;
	getchar();
	return 0;
}