#include "odcs.hpp"
#include <Eigen\Geometry>
#include <iostream>
#include <fstream>
#include <vector>
#include <Windows.h>
#include <opencv2/core.hpp>
#include <opencv2/shape.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#pragma comment(lib, "winmm.lib")

int main()
{
	std::ofstream ofs("det_roi_simple.csv");
	//ofs << "t, x, y, z, isTracked" << std::endl;
	odcs odcs;
	std::cout << "ODCS Initializing..." << std::endl;
	odcs.Initialize();
	std::vector<FloatingObjectPtr> objPtrs;
	objPtrs.push_back(FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1350))));
	//objs.push_back(FloatingObject(Eigen::Vector3f(-250, 100, 1485))); // add object
	odcs.StartControl(objPtrs);

	std::cout << "Press any key to close." << std::endl;
	getchar();
	odcs.Close();
	return 0;
}
