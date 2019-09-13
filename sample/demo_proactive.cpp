#include "odcs.hpp"
#include "additionalGain.hpp"
#include "winMultiplexer.hpp"
#include "autd3.hpp"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <thread>
#include <chrono>
#include <mutex>
#define NOMINMAX
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

#pragma comment(lib, "winmm")

using namespace dynaman;

int main() {

	/*デモするときに調整するパラメータ*/
	Eigen::Vector3f posRight(400.14f, 596.04f, 1331.f);//右に移動した後の位置
	Eigen::Vector3f posLeft(-400.86f, 596.04f, 1331.f);//左に移動した後の位置
	double translationTime = 2.0; //移動にかける時間(短いほど早い、安定性は下がる)
	double intervalTime = 4.0; //移動後の待ち時間

	odcs dynaman;
	dynaman.Initialize();
	dynaman.odsPtr->SetWorkSpace(Eigen::Vector3f(-800.f, 0.f, 500.f), Eigen::Vector3f(800.f, 1000.f, 1570.f));
	Eigen::Matrix3f rotationKinect2Global;
	rotationKinect2Global <<
		-0.999986, -0.000739038, 0.00521048,
		0.00518096, 0.0355078, 0.999356,
		-0.000923425, 0.999369, -0.0355035;
	dynaman.odsPtr->SetSensorGeometry(Eigen::Vector3f(35.1867f, -1242.32f, 1085.62f), rotationKinect2Global);

	FloatingObjectPtr objPtr = FloatingObject::Create(posLeft, -0.0001f, 79.0f);

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
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(15)); // wait until the floating object is stabilized.

	winMultiplexer::orders_vector orders{
		std::make_pair([&]() {objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, posLeft, posRight))); std::cout << "translation started" << std::endl; } , 1000*(translationTime + intervalTime)),
		std::make_pair([&]() {objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, posRight, posLeft))); std::cout << "translation started" << std::endl; } , 1000*(translationTime + intervalTime)),
	};
	winMultiplexer multiplexer(orders);
	multiplexer.RunAsync();
	getchar();
	multiplexer.Stop();
	dynaman.Close();
}