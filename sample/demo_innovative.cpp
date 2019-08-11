#include "odcs.hpp"
#include "additionalGain.hpp"
#include "init_uist.hpp"
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

int main() {

	/*デモするときに調整するパラメータ*/
	Eigen::Vector3f posCenter(0.0f, 496.04f, 1031.f);
	Eigen::Vector3f posRight(300.14f, 596.04f, 1331.f);//右に移動した後の位置
	Eigen::Vector3f posLeft(-300.86f, 596.04f, 1031.f);//左に移動した後の位置
	Eigen::Vector3f posRight2(380.14f, 596.04f, 1331.f);//右に移動した後の位置

	double translationTime = 10.0; //移動にかける時間(短いほど早い、安定性は下がる)
	double intervalTime = 5.0; //移動後の待ち時間

	odcs dynaman;
	initialize_uist_setup(dynaman);
	//control parameters
	Eigen::Vector3f gainP = 1.0f*Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = 1.0f*Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = 1.0f*Eigen::Vector3f::Constant(-0.05f);

	FloatingObjectPtr objPtr = FloatingObject::Create(posCenter, -0.0001f, 127.f);
	dynaman.RegisterObject(objPtr);
	dynaman.StartControl();
	
	std::thread t_log([&objPtr]() {
		std::ofstream ofs("20190703_10inch_woHenshin6.csv");
		DWORD tInit = timeGetTime();
		ofs << "time[ms], x[mm], y[mm], z[mm], vx[mm/s], vy[mm/s], vz[mm/s], Ix[mm s], Iy[mm s], Iz[mm s], xTgt[mm], yTgt[mm], zTgt[mm], vxTgt[mm/s], vyTgt[mm/s], vzTgt[mm/s] " << std::endl;
		while (timeGetTime() - tInit < 60000) {
			DWORD currentTime = timeGetTime() - tInit;
			Eigen::Vector3f pos = objPtr->getPosition();
			Eigen::Vector3f vel = objPtr->getVelocity();
			Eigen::Vector3f integral = objPtr->getIntegral();
			Eigen::Vector3f posTgt = objPtr->getPositionTarget();
			Eigen::Vector3f velTgt = objPtr->getVelocityTarget();
			ofs << currentTime << ", " << pos.x() << ", " << pos.y() << ", " << pos.z() << ", " << vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
				<< integral.x() << ", " << integral.y() << ", " << integral.z() << ", "
				<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", " << velTgt.x() << ", " << velTgt.y() << ", " << velTgt.z() << std::endl;
				
			std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(30));
		}
		std::cout << "press enter to close." << std::endl;
	});
	
	
	//std::cout << "press any key to start translation." << std::endl;

	getchar();
	/*
	//std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(5)); // wait until the floating object is stabilized.
	objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(6.0f, timeGetTime() / 1000.f, posRight, posLeft)));
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(8)); // wait until the floating object is stabilized.
	objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(6.0f, timeGetTime() / 1000.f, posLeft, posRight)));
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(8)); // wait until the floating object is stabilized.
	std::cout << "press any key to proceed." << std::endl;
	getchar();

	*/
	t_log.join();
	
	dynaman.Close();
}