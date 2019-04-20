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

#pragma comment(lib, "winmm.lib")

int main()
{
	Eigen::Vector3f center(0, 596.04f, 1331.f);
	Eigen::Vector3f posRight1(426.14f, 596.04f + 160.f, 1231.f);//右に移動した後の位置
	Eigen::Vector3f posLeft1(-423.86f, 596.04f + 160.f, 1231.f);//左に移動した後の位置
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
	Eigen::Vector3f gainP = Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = Eigen::Vector3f::Constant(-0.05f);
	dynaman.ocsPtr->SetGain(gainP, gainD, gainI);

	//odcs.ocs.SetGain(Eigen::Vector3f::Constant(-1.6f), Eigen::Vector3f::Constant(-2.6f), Eigen::Vector3f::Constant(-0.36f));
	FloatingObjectPtr objPtr1 = FloatingObject::Create(center, -0.0001f);
	FloatingObjectPtr objPtr2 = FloatingObject::Create(posRight2, -0.0001f);
	dynaman.RegisterObject(objPtr1); // add object
	//dynaman.RegisterObject(objPtr2); // add object
	dynaman.StartControl();	
	std::thread th_log([&objPtr1, &objPtr2]() {
		std::ofstream ofs("20190405_log_single.csv");
		DWORD timeInit = timeGetTime();
		while (1) {
			DWORD currentTime = timeGetTime();
			Eigen::Vector3f pos1 = objPtr1->getPosition();
			Eigen::Vector3f posTgt1 = objPtr1->getPositionTarget();
			Eigen::Vector3f velTgt1 = objPtr1->getVelocityTarget();
			Eigen::Vector3f accelTgt1 = objPtr1->getVelocityTarget();

			Eigen::Vector3f pos2 = objPtr2->getPosition();
			Eigen::Vector3f posTgt2 = objPtr2->getPositionTarget();
			Eigen::Vector3f velTgt2 = objPtr2->getVelocityTarget();
			Eigen::Vector3f accelTgt2 = objPtr2->getVelocityTarget();

			ofs << currentTime << ", " << pos1.x() << "," << pos1.y() << "," << pos1.z() << ","
				<< posTgt1.x() << "," << posTgt1.y() << "," << posTgt1.z() << ","
				<< velTgt1.x() << "," << velTgt1.y() << "," << velTgt1.z() << ","
				<< accelTgt1.x() << "," << accelTgt1.y() << "," << accelTgt1.z() << ","
				<< pos2.x() << "," << pos2.y() << "," << pos2.z() << ", "
				<< posTgt2.x() << "," << posTgt2.y() << "," << posTgt2.z() << ","
				<< velTgt2.x() << "," << velTgt2.y() << "," << velTgt2.z() << ","
				<< accelTgt2.x() << "," << accelTgt2.y() << "," << accelTgt2.z() << std::endl;
			if (currentTime - timeInit > 130000) break;
			std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(40));
		}
	});
	//std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(60));
	//objPtr1->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(10.0f, timeGetTime() / 1000.f, posLeft1, posRight1)));
	//objPtr2->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(10.0f, timeGetTime() / 1000.f, posRight2, posLeft2)));
	std::cout << "Press any key to close." << std::endl;
	getchar();
	dynaman.Close();
	th_log.join();
	return 0;
}
