#include "Projector.hpp"
#include "odcs.hpp"
#include "arfModel.hpp"
#include <Eigen\Geometry>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

#pragma comment(lib, "winmm.lib")

int main()
{
	std::cout << "Initializing..." << std::endl;
	odcs odcs;
	odcs.Initialize();
	std::vector<FloatingObjectPtr> objPtrs;
	objPtrs.push_back(FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1350))));
	//objs.push_back(FloatingObject(Eigen::Vector3f(-250, 100, 1485)));

	odcs.StartControl(objPtrs);
	
	//write your application process here.
	Sleep(10000);
	objPtrs[0]->updateStatesTarget(Eigen::Vector3f(300, 0, 1350), Eigen::Vector3f(0, 0, 0));
	Sleep(10000);
	objPtrs[0]->updateStatesTarget(Eigen::Vector3f(-300, 0, 1350), Eigen::Vector3f(0, 0, 0));
	
	std::cout << "Press any key to close." << std::endl;
	getchar();
	odcs.Close();
	return 0;
}
