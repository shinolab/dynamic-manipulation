#include "odcs.hpp"
#include <Eigen\Geometry>
#include <iostream>
#include <vector>
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

#pragma comment(lib, "winmm.lib")

int main()
{
	odcs odcs;
	std::cout << "ODCS Initializing..." << std::endl;
	odcs.Initialize();
	std::vector<FloatingObjectPtr> objPtrs;
	objPtrs.push_back(FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1350))));
	//objs.push_back(FloatingObject(Eigen::Vector3f(-250, 100, 1485)));

	odcs.StartControl(objPtrs);

	//write your application process here.
	objPtrs[0]->updateStatesTarget(Eigen::Vector3f(200, 0, 1350), Eigen::Vector3f(0, 0, 0)); //Move Object
	Sleep(10000);
	objPtrs[0]->updateStatesTarget(Eigen::Vector3f(0, 0, 1350), Eigen::Vector3f(0, 0, 0)); //Move Object

	std::cout << "Press any key to close." << std::endl;
	getchar();
	odcs.Close();
	return 0;
}
