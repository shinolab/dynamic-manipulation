#include "odcs.hpp"
#include "kalmanFilter.hpp"
#include "profile.hpp"
#include <Eigen\Geometry>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <Windows.h>
#include <opencv2/core.hpp>
#include <opencv2/shape.hpp>
#include <opencv2/highgui.hpp>

#pragma comment(lib, "winmm.lib")

int main()
{
	Eigen::Vector3f center(0, 596.04f, 1331.f);
	Eigen::Vector3f posRight1(426.14f, 596.04f + 160.f, 1331.f);//右に移動した後の位置
	Eigen::Vector3f posLeft1(-423.86f, 596.04f+100, 1431.f);//左に移動した後の位置
	Eigen::Vector3f posRight2(426.14f, 596.04f - 160.f, 1431.f);//右に移動した後の位置
	Eigen::Vector3f posLeft2(-423.86f, 596.04f - 160.f, 1431.f);//左に移動した後の位置

	odcs dynaman;
	dynaman.Initialize();
	dynaman.odsPtr->SetWorkSpace(Eigen::Vector3f(-800.f, 0.f, 500.f), Eigen::Vector3f(800.f, 1000.f, 1570.f));
	Eigen::Matrix3f rotationKinect2Global;
	rotationKinect2Global <<
		-0.999986, -0.000739038, 0.00521048,
		0.00518096, 0.0355078, 0.999356,
		-0.000923425, 0.999369, -0.0355035;
	dynaman.odsPtr->SetSensorGeometry(Eigen::Vector3f(35.1867f, -1242.32f, 1085.62f), rotationKinect2Global);

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
	Eigen::Vector3f gainP = 1.0f*Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = 1.0f*Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = 0.0f*Eigen::Vector3f::Constant(-0.05f);
	dynaman.ocsPtr->SetGain(gainP, gainD, gainI);
	//odcs.ocs.SetGain(Eigen::Vector3f::Constant(-1.6f), Eigen::Vector3f::Constant(-2.6f), Eigen::Vector3f::Constant(-0.36f));
	FloatingObjectPtr objPtr = FloatingObject::Create(center, -0.0001f, 178);
	dynaman.RegisterObject(objPtr); // add object
	dynaman.StartControl();

	std::cout << "Press any key to close." << std::endl;

	getchar();
	dynaman.Close();
	return 0;
}
