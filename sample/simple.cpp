#include "odcs.hpp"
#include "kalmanFilter.hpp"
#include "profile.hpp"
#include <Eigen\Geometry>
#include <iostream>
#include <fstream>
#include <vector>
#include <Windows.h>
#include <opencv2/core.hpp>
#include <opencv2/shape.hpp>

#pragma comment(lib, "winmm.lib")

int main()
{
	odcs odcs;
	std::cout << "ODCS Initializing..." << std::endl;
	odcs.Initialize();
	//odcs.ocs.SetGain(Eigen::Vector3f::Constant(-1.6f), Eigen::Vector3f::Constant(-2.6f), Eigen::Vector3f::Constant(-0.36f));
	Eigen::Vector3f posDefault(50.f, 0.f, 1300.f);
	FloatingObjectPtr objPtr(new FloatingObject(posDefault));
	odcs.RegisterObject(objPtr); // add object
	odcs.ocsPtr->SetGain(Eigen::Vector3f::Constant(-1.6f), Eigen::Vector3f::Constant(-4.0f), Eigen::Vector3f::Constant(-0.1f));
	odcs.StartControl();
	std::thread threadLog([&odcs]() {
		std::ofstream ofs("20190213_mov350.csv");
		ofs << "t, x, y, z, vx,vy, vz, xTgt, yTgt, zTgt, vxTgt, vyTgt, vzTgt" << std::endl;
		while (1)
		{
			Eigen::Vector3f pos = odcs.GetFloatingObject(0)->getPosition();
			Eigen::Vector3f vel = odcs.GetFloatingObject(0)->getVelocity();
			Eigen::Vector3f posTgt = odcs.GetFloatingObject(0)->getPositionTarget();
			Eigen::Vector3f velTgt = odcs.GetFloatingObject(0)->getVelocityTarget();
			ofs << timeGetTime() / 1000.f << ", " << pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
				<< vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
				<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", "
				<< velTgt.x() << ", " << velTgt.y() << ", " << velTgt.z() << std::endl;
			Sleep(30);
		}
	});
	Eigen::Vector3f posRight(350.f, 0.f, 1300.f);
	Eigen::Vector3f posLeft(-250.f, 0.f, 1300.f);
	Sleep(30000);
	//move to right
	std::cout << "MOVE TO RIGHT." << std::endl;
	float dt = 4.0f;
	float t1 = timeGetTime() / 1000.f;
	profileBangBang profile1(dt, t1, posDefault, posRight);
	while (timeGetTime() / 1000.f - t1 < dt + 1)
	{
		objPtr->updateStatesTarget(profile1.posTgt(), profile1.velTgt(), profile1.accelTgt());
	}
	std::cout << "MOVE TO LEFT." << std::endl;
	float t2 = timeGetTime() / 1000.f;
	profileBangBang profile2(2*dt, t2, posRight, posLeft);
	while (timeGetTime() / 1000.f - t2 < 2*dt + 1)
	{
		objPtr->updateStatesTarget(profile2.posTgt(), profile2.velTgt(), profile2.accelTgt());
	}
	std::cout << "MOVE TO RIGHT." << std::endl;

	float t3 = timeGetTime() / 1000.f;
	profileBangBang profile3(2*dt, t3, posLeft, posRight);
	while (timeGetTime() / 1000.f - t3 < 2*dt + 1)
	{
		objPtr->updateStatesTarget(profile3.posTgt(), profile3.velTgt(), profile3.accelTgt());
	}
	std::cout << "CIRCLE TRAJ." << std::endl;

	float t4 = timeGetTime() / 1000.f;
	profileCircle profileC(posDefault, 300.f, 5*dt, t4);
	while (timeGetTime() / 1000.f - t4 < 2.5*dt)
	{
		objPtr->updateStatesTarget(profileC.posTgt(), profileC.velTgt(), profileC.accelTgt());
	}
	objPtr->updateStatesTarget(posDefault, Eigen::Vector3f(0, 0, 0));


	//Sleep(10000);
	//objPtr->updateStatesTarget(Eigen::Vector3f(20, 25, 1140), Eigen::Vector3f(0, 0, 0));
	//Sleep(20000);
	//objPtr->updateStatesTarget(Eigen::Vector3f(43, 1, 1000), Eigen::Vector3f(0, 0, 0));
	std::cout << "Press any key to close." << std::endl;
	getchar();
	odcs.Close();
	return 0;
}
