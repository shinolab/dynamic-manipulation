#include "odcs.hpp"
#include "KinectSphereTracker.hpp"
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

	//initialization of the sensor
	Eigen::Matrix3f rotationKinect2Global;
	rotationKinect2Global <<
		-0.999964f, 0.00838503f, 0.00165846f,
		0.00108414f, -0.0680396f, 0.997682f,
		0.00847844f, 0.997648f, 0.068028f;
	std::cout << "constructing sensor ... " << std::endl;
	dynaman::KinectDepthSphereTracker sensor(Eigen::Vector3f(44.9728f, -1200.87f, 1086.58f),
		Eigen::Quaternionf(rotationKinect2Global),
		true);

	std::cout << "sensor initialized" << std::endl;
	//initialization of dynaman
	dynaman::odcs dynaman(sensor);
	dynaman.Initialize();

	std::cout << "adding devices..." << std::endl;
	dynaman.AddDevice(Eigen::Vector3f(-992.5f, 790.f + 122.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(-992.5f, 270.f + 122.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(-542.5f, 1050.f + 122.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(-542.5f, 530.f + 122.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(-542.5f, 10.f + 122.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(-92.5f, 790.f + 122.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(-92.5f, 270.f + 122.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(357.5f, 1050.f + 122.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(357.5f, 530.f + 122.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(357.5f, 10.f + 122.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(807.5f, 790.f + 122.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(807.5f, 270.f + 122.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	/*
	dynaman.AddDevice(Eigen::Vector3f(-807.5f, 790.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(-807.5f, 270.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(-357.5f, 1050.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(-357.5f, 530.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(-357.5f, 10.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(92.5f, 790.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(92.5f, 270.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(542.5f, 1050.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(542.5f, 530.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(542.5f, 10.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(992.5f, 790.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	dynaman.AddDevice(Eigen::Vector3f(992.5f, 270.f, 1911.f), Eigen::Vector3f(0.f, M_PI, M_PI));
	*/
	
	std::cout << "applying control gains..." << std::endl;
	//control parameters
	Eigen::Vector3f gainP = 1.0f*Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = 1.0f*Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = 1.0f*Eigen::Vector3f::Constant(-0.05f);
	dynaman.ocsPtr->SetGain(gainP, gainD, gainI);
	Eigen::Vector3f lowerbound(-800.f, 0.f, 500.f);
	Eigen::Vector3f upperbound(800.f, 1200.f, 1570.f);
	//odcs.ocs.SetGain(Eigen::Vector3f::Constant(-1.6f), Eigen::Vector3f::Constant(-2.6f), Eigen::Vector3f::Constant(-0.36f));
	std::cout << "Creating floating objects..." << std::endl;
	auto objPtr1 = dynaman::FloatingObject::Create(posLeft1, lowerbound, upperbound, -0.0001f, 127.f);
	//FloatingObjectPtr objPtr2 = FloatingObject::Create(posRight2, -0.0001f);
	dynaman.RegisterObject(objPtr1); // add object
	//dynaman.RegisterObject(objPtr2); // add object
	std::cout << "starting control..." << std::endl;
	dynaman.StartControl();	
	//objPtr2->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(10.0f, timeGetTime() / 1000.f, posRight2, posLeft2)));


	std::thread t_log([&objPtr1]() {
		std::ofstream ofs("20191112_log.csv");
		DWORD t_init = timeGetTime();
		while (timeGetTime() - t_init < 12000) {
			Eigen::Vector3f position = objPtr1->getPosition();
			Eigen::Vector3f positionTarget = objPtr1->getPositionTarget();
			ofs << timeGetTime() << ", " << position.x() << ", " << position.y() << ", " << position.z() << ", " << positionTarget.x() << ", " << positionTarget.y() << ", " << positionTarget.z() << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(33));
		}

		ofs.close();
		}
	);

	std::cout << "Press any key to start translation." << std::endl;
	getchar();
	objPtr1->SetTrajectory(std::shared_ptr<dynaman::Trajectory>(new dynaman::TrajectoryBangBang(5.0f, timeGetTime() / 1000.f, posLeft1, posRight1)));

	t_log.join();
	std::cout << "Press any key to close." << std::endl;
	getchar();
	dynaman.Close();

	std::cout << "dynaman has been closed." << std::endl;

	return 0;
}
