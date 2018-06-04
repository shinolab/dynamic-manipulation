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
	//ods ods; ods.Initialize();
	//ocs ocs; ocs.Initialize();
	std::cout << "Initializing..." << std::endl;
	odcs odcs;
	odcs.Initialize();
	std::vector<FloatingObjectPtr> objPtrs;
	objPtrs.push_back(FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(0,0,1350))));
	//objs.push_back(FloatingObject(Eigen::Vector3f(-250, 100, 1485)));
	
	//cv::imshow("controlwindow", cv::Mat::zeros(500, 500, CV_8UC1));
	odcs.StartControl(objPtrs);

	for (int i = 0; i< 10; i++)
	{
		std::cout << i << "sec" << std::endl;
		Sleep(1000);
	}
	/*
	while (1)
	{
	odcs.ControlLoop(objPtrs);
	auto key = cv::waitKey(1);
	if (key == 'q')
	{
	cv::destroyAllWindows();
	break;
	}
	}
	*/
	
	std::cout << "Press any key to close." << std::endl;

	getchar();
	odcs.Close();
	return 0;
		
}
