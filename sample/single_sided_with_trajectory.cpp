#include "odcs.hpp"
#include "additionalGain.hpp"
#include "autd3.hpp"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#define NOMINMAX
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

#pragma comment(lib, "winmm")

int main() {
	odcs dynaman;
	dynaman.Initialize();
	dynaman.odsPtr->SetWorkSpace(Eigen::Vector3f(-800.f, 0.f, 500.f), Eigen::Vector3f(800.f, 1000.f, 1500.f));
	dynaman.odsPtr->SetSensorGeometry(Eigen::Vector3f(40.f, -1221.f, 1080.f), Eigen::Vector3f(M_PI_2, M_PI_2, M_PI_2));
	
	FloatingObjectPtr objPtr = FloatingObject::Create(Eigen::Vector3f(540.f, 580.f, 1360.f), -0.001f);

	dynaman.AddDevice(Eigen::Vector3f(992.5f, 270.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(992.5f, 790.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(542.5f, 10.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(542.5f, 530.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(542.5f, 1050.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(92.5f, 270.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(92.5f, 790.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(-357.5f, 10.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(-357.5f, 530.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(-357.5f, 1050.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(-807.5f, 270.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(-807.5f, 790.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));

	//control parameters
	Eigen::Vector3f gainP = Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = Eigen::Vector3f::Constant(-0.05f);
	dynaman.ocsPtr->SetGain(gainP, gainD, gainI);
	dynaman.RegisterObject(objPtr);
	dynaman.StartControl();
	std::thread th_log([&objPtr]() {
		std::ofstream ofs_log("0190318_trajectory_log.csv");
		ofs_log << "t, x, y, z, xTgt, yTgt, zTgt, vxTgt, vyTgt, vzTgt, axTgt, ayTgt, azTgt" << std::endl;
		DWORD timeLogInit = timeGetTime();
		while (timeGetTime() - timeLogInit < 50000) {
			ofs_log << timeGetTime() << ", " << objPtr->getPosition().x() << ", " << objPtr->getPosition().y() << ", " << objPtr->getPosition().z() << ", "
				<< objPtr->getPositionTarget().x() << ", " << objPtr->getPositionTarget().y() << ", " << objPtr->getPositionTarget().z() << ", "
				<< objPtr->getVelocityTarget().x() << ", " << objPtr->getVelocityTarget().y() << ", " << objPtr->getVelocityTarget().z() << ", "
				<< objPtr->getAccelTarget().x() << ", " << objPtr->getAccelTarget().y() << ", " << objPtr->getAccelTarget().z() << std::endl;
			Sleep(30);
		}
	});

	//objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(5.0f, timeGetTime() / 1000.f + 20.f, objPtr->getPositionTarget(), Eigen::Vector3f(86, 580, 600))));
	std::cout << "Translation set." << std::endl;
	th_log.join();
	dynaman.Close();
	return 0;
}