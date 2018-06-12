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
	objPtrs.push_back(FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(0, 200, 1500))));
	//objs.push_back(FloatingObject(Eigen::Vector3f(-250, 100, 1485)));
	odcs.StartControl(objPtrs);
	/*
	while (1)
	{
	odcs.ods.DeterminePositionByDepthWithROI(objPtrs[0]);
	//odcs.ods.DeterminePositionByDepth(objPtrs);
	ofs << timeGetTime() - initialTime << ", "
	<< objPtrs[0]->getPosition().x() << ","
	<< objPtrs[0]->getPosition().y() << ","
	<< objPtrs[0]->getPosition().z() << ","
	<< objPtrs[0]->isTracked << std::endl;

	//std::cout << objPtrs[0]->isTracked << std::endl;
	auto key = cv::waitKey(1);
	if (key == 'q')
	{
	cv::destroyAllWindows();
	break;
	}
	}
	*/
	
	//write your application process here.
	//objPtrs[0]->updateStatesTarget(Eigen::Vector3f(200, 0, 1350), Eigen::Vector3f(0, 0, 0)); //Move Object
	//Sleep(10000);
	//objPtrs[0]->updateStatesTarget(Eigen::Vector3f(0, 0, 1350), Eigen::Vector3f(0, 0, 0)); //Move Object

	std::cout << "Press any key to close." << std::endl;
	getchar();
	odcs.Close();
	return 0;
}
