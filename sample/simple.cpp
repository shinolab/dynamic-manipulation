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

	Eigen::Vector3f positionAUTD0(-85, -65, 0); Eigen::Vector3f eulerAngleAUTD0(0, 0, 0);
	Eigen::Vector3f positionAUTD1(-998, 65, 1038); Eigen::Vector3f eulerAngleAUTD1(M_PI, -M_PI_2, 0);
	Eigen::Vector3f positionAUTD2(998, -65, 1038); Eigen::Vector3f eulerAngleAUTD2(0, -M_PI_2, 0);
	Eigen::Vector3f positionAUTD3(-65, -998, 1038); Eigen::Vector3f eulerAngleAUTD3(-M_PI_2, -M_PI_2, 0);
	Eigen::Vector3f positionAUTD4(65, 998, 1038); Eigen::Vector3f eulerAngleAUTD4(M_PI_2, -M_PI_2, 0);

	odcs.AddDevice(positionAUTD0, eulerAngleAUTD0);
	odcs.AddDevice(positionAUTD1, eulerAngleAUTD1);
	odcs.AddDevice(positionAUTD2, eulerAngleAUTD2);
	odcs.AddDevice(positionAUTD3, eulerAngleAUTD3);
	odcs.AddDevice(positionAUTD4, eulerAngleAUTD4);

	//odcs.ocs.SetGain(Eigen::Vector3f::Constant(-1.6f), Eigen::Vector3f::Constant(-2.6f), Eigen::Vector3f::Constant(-0.36f));
	Eigen::Vector3f posDefault(50.f, 0.f, 1300.f);
	FloatingObjectPtr objPtr(new FloatingObject(posDefault));
	odcs.RegisterObject(objPtr); // add object
	odcs.ocsPtr->SetGain(Eigen::Vector3f::Constant(-1.6f), Eigen::Vector3f::Constant(-4.0f), Eigen::Vector3f::Constant(-0.05f));
	odcs.StartControl();	
	std::cout << "Press any key to close." << std::endl;
	getchar();
	odcs.Close();
	return 0;
}
