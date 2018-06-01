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
	ods ods; ods.Initialize();
	ocs ocs; ocs.Initialize();
	cv::imshow("controlwindow", cv::Mat::zeros(500, 500, CV_8UC1));
	std::cout << "Initializing..." << std::endl;
	std::vector<FloatingObject> objs;
	objs.push_back(FloatingObject(Eigen::Vector3f(0, 0, 1350)));
	//objs.push_back(FloatingObject(Eigen::Vector3f(-250, 100, 1485)));

	while (1)
	{
		for (auto itr = objs.begin(); itr != objs.end(); itr++)
		{
			ods.DeterminePositionByDepth(objs);
						
			Eigen::VectorXf duties = ocs.FindDutyQP(&(*itr)) / objs.size();
			//Eigen::VectorXi amplitudes = (255 * duties).cwiseMax(0).cwiseMin(255).cast<int>();
			Eigen::VectorXi amplitudes = (510 / M_PI * duties.array().sqrt().asin().max(0).min(255)).matrix().cast<int>();
			ocs.DirectSemiPlaneWave(&(*itr), amplitudes);
		}
		auto key = cv::waitKey(1);
		if (key == 'q')
		{
			cv::destroyAllWindows();
			break;
		}
	}
	std::cout << "Press any key to close." << std::endl;

	getchar();
	ocs.Close();
	return 0;
		
}
