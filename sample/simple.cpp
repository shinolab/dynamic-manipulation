#include "odcs.hpp"
#include "kalmanFilter.hpp"
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
	FloatingObjectPtr objPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1350)));
	odcs.RegisterObject(objPtr); // add object
	odcs.StartControl();
	Sleep(10000);
	objPtr->updateStatesTarget(Eigen::Vector3f(20, 25, 1140), Eigen::Vector3f(0, 0, 0));
	//Sleep(20000);
	//objPtr->updateStatesTarget(Eigen::Vector3f(43, 1, 1000), Eigen::Vector3f(0, 0, 0));
	std::cout << "Press any key to close." << std::endl;
	getchar();
	odcs.Close();
	return 0;
}
