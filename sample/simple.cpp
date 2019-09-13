#include "odcs.hpp"
#include "kalmanFilter.hpp"
#include <Eigen\Geometry>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <Windows.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#pragma comment(lib, "winmm.lib")

int main()
{
	Eigen::Vector3f center(0, 596.04f, 1331.f);
	Eigen::Vector3f posRight1(426.14f, 596.04f, 1331.f);//右に移動した後の位置
	Eigen::Vector3f posLeft1(-423.86f, 596.04f, 1431.f);//左に移動した後の位置
	Eigen::Vector3f posRight2(426.14f, 596.04f - 160.f, 1431.f);//右に移動した後の位置
	Eigen::Vector3f posLeft2(-423.86f, 596.04f - 160.f, 1431.f);//左に移動した後の位置

	dynaman::odcs dynaman;
	dynaman.Initialize();
	dynaman.odsPtr->SetWorkSpace(Eigen::Vector3f(-800.f, 0.f, 500.f), Eigen::Vector3f(800.f, 1000.f, 1570.f));
	Eigen::Matrix3f rotationKinect2Global;
	rotationKinect2Global <<
		-0.999971f,  0.00739537f, -0.00169575f,
		-0.001628f, 0.0091623f, 0.999957f,
		0.00741063f, 0.999931f, -0.00915001f;
	Eigen::Matrix3f rot2 = rotationKinect2Global;
	dynaman.odsPtr->SetSensorGeometry(Eigen::Vector3f(38.5924f, -1244.1f, 1087.02f), rot2);
	std::cout << "adding devices..." << std::endl;
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
	std::cout << "applying control gains..." << std::endl;
	//control parameters
	Eigen::Vector3f gainP = 1.0f*Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = 1.0f*Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = 1.0f*Eigen::Vector3f::Constant(-0.05f);
	dynaman.ocsPtr->SetGain(gainP, gainD, gainI);
	//odcs.ocs.SetGain(Eigen::Vector3f::Constant(-1.6f), Eigen::Vector3f::Constant(-2.6f), Eigen::Vector3f::Constant(-0.36f));
	std::cout << "Creating floating objects..." << std::endl;
	auto objPtr1 = dynaman::FloatingObject::Create(posLeft1, -0.0001f, 127.f);
	//FloatingObjectPtr objPtr2 = FloatingObject::Create(posRight2, -0.0001f);
	dynaman.RegisterObject(objPtr1); // add object
	//dynaman.RegisterObject(objPtr2); // add object
	std::cout << "starting control..." << std::endl;
	dynaman.StartControl();	
	getchar();
	std::cout << "Press any key to start translation." << std::endl;
	objPtr1->SetTrajectory(std::shared_ptr<dynaman::Trajectory>(new dynaman::TrajectoryBangBang(5.0f, timeGetTime() / 1000.f, posLeft1, posRight1)));
	//objPtr2->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(10.0f, timeGetTime() / 1000.f, posRight2, posLeft2)));

	std::cout << "Press any key to close." << std::endl;
	getchar();
	dynaman.Close();
	return 0;
}
