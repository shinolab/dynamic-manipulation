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
	odcs.ocsPtr->SetGain(Eigen::Vector3f::Constant(-1.6f), Eigen::Vector3f::Constant(-4.0f), Eigen::Vector3f::Constant(-0.05f));
	odcs.StartControl();	
	std::cout << "Press any key to close." << std::endl;
	getchar();
	odcs.Close();
	return 0;
}
