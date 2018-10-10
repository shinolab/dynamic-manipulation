#define NOMINMAX
#include "projector.hpp"
#include "odcs.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <random>

int main()
{
	const int numOptoPerLine = 5;
	const int numFailedCriteria = 2;
	std::string name = "testee_name";
	//Step 0: initialization
	std::cout << "Press enter to capture the background including a tripod." << std::endl;
	std::getline(std::cin, std::string());
	odcs odcs;
	odcs.Initialize();
	FloatingObjectPtr objPtr = FloatingObject::Create(Eigen::Vector3f(0, 0, 1300));
	std::cout << "Press any key to determine a default position of the balloon." << std::endl;
	std::getline(std::cin, std::string());
	//Step 1: Determine Position on a tripod.
	Eigen::Vector3f posTgt;
	while (1)
	{
		bool initSuccessed = odcs.ods.GetPositionByDepth(objPtr, posTgt, false);
		if (initSuccessed)
		{
			break;
		}
		Sleep(10);
	}
	objPtr->updateStatesTarget(posTgt, Eigen::Vector3f(0, 0, 0));
	DWORD observationTime = timeGetTime();
	objPtr->updateStates(observationTime, posTgt);
	objPtr->isTracked = true;

	//Step 2: Start visual acuity test(static)
	projector proj("projecor");
	std::random_device rng;
	cv::Mat LandoltCUp = cv::imread("img/LandoltC_up.bmp");
	cv::Mat LandoltCDown = cv::imread("img/LandoltC_down.bmp");
	cv::Mat LandoltCRight = cv::imread("img/LandoltC_right.bmp");
	cv::Mat LandoltCLeft = cv::imread("img/LandoltC_left.bmp");
	cv::Mat LandoltO = cv::imread("img/LandoltC");

	//static image test
	std::string fileName = name + "_static_test.log";
	std::ofstream ofsS(fileName);
	bool failed = false;
	bool finished = false;
	unsigned int count_failed = 0;
	int try_count = 0;
	int tenth_acuity = 10;
	int distance = 2000;
	while (!finished && !(tenth_acuity == 0))
	{
		proj.projectImageOnObject(posTgt, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(255)), cv::Size(100, 100));
		cv::waitKey(1000);
		float size = 5 * distance * tanf(10.0 * M_PI / 3 / tenth_acuity);
		int direction = rng() % 4;
		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(posTgt, LandoltCUp, cv::Size(size, size));
			break;
		case 1:
			proj.projectImageOnObject(posTgt, LandoltCDown, cv::Size(size, size));
			break;
		case 2:
			proj.projectImageOnObject(posTgt, LandoltCRight, cv::Size(size, size));
			break;
		case 3:
			proj.projectImageOnObject(posTgt, LandoltCLeft, cv::Size(size, size));
			break;
		default:
			proj.projectImageOnObject(posTgt, LandoltO, cv::Size(size, size));
			break;
		}
		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 65361:
			answer = 3;
			break;
		case 65362:
			answer = 0;
			break;
		case 65363:
			answer = 2;
			break;
		case 65364:
			answer = 1;
			break;
		default:
			break;
		}
		ofsS << tenth_acuity / 10.0 << direction << "," << answer << std::endl;
		try_count++;
		if (answer != direction)
		{
			count_failed++;
		}
		if (try_count == numOptoPerLine)
		{
			if (failed)
			{
				finished = true;
			}
			if (count_failed >= numFailedCriteria)
			{
				failed = true;
			}
			count_failed = 0;
			tenth_acuity--; 
		}
	}
	ofsS.close();
	std::cout << "Static Test Ended.\n Please remove the tripod. Then, press any key to begin dynamic acuity test." << std::endl;
	odcs.RegisterObject(objPtr);
	odcs.StartControl();
	tenth_acuity = 10;
	std::ofstream ofsD1(name + "_dynamic_test1.log");
	ofsD1 << "acuity, true direction, answer" << std::endl;
	failed = false;
	finished = false;
	count_failed = 0;
	try_count = 0;
	tenth_acuity = 10;
	while (!finished && !(tenth_acuity == 0))
	{
		proj.projectImageOnObject(posTgt, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(255)), cv::Size(100, 100));
		cv::waitKey(1000);
		float size = 5 * distance * tanf(10.0 * M_PI / 3 / tenth_acuity);
		int direction = rng() % 4;
		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(posTgt, LandoltCUp, cv::Size(size, size));
			break;
		case 1:
			proj.projectImageOnObject(posTgt, LandoltCDown, cv::Size(size, size));
			break;
		case 2:
			proj.projectImageOnObject(posTgt, LandoltCRight, cv::Size(size, size));
			break;
		case 3:
			proj.projectImageOnObject(posTgt, LandoltCLeft, cv::Size(size, size));
			break;
		default:
			proj.projectImageOnObject(posTgt, LandoltO, cv::Size(size, size));
			break;
		}
		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 65361:
			answer = 3;
			break;
		case 65362:
			answer = 0;
			break;
		case 65363:
			answer = 2;
			break;
		case 65364:
			answer = 1;
			break;
		default:
			break;
		}
		ofsD1 << tenth_acuity / 10.0 << direction << "," << answer << std::endl;
		try_count++;
		if (answer != direction)
		{
			count_failed++;
		}
		if (try_count == numOptoPerLine)
		{
			if (failed)
			{
				finished = true;
			}
			if (count_failed >= numFailedCriteria)
			{
				failed = true;
			}
			count_failed = 0;
			tenth_acuity--;
		}
	}
	ofsD1.close();
	std::ofstream ofsD2(name + "_dynamic_test2.log");
	ofsD2 << "acuity, true direction, answer" << std::endl;
	failed = false;
	finished = false;
	count_failed = 0;
	try_count = 0;
	tenth_acuity = 10;
	while (!finished && !(tenth_acuity == 0))
	{
		proj.projectImageOnObject(posTgt, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(255)), cv::Size(100, 100));
		cv::waitKey(1000);
		float size = 5 * distance * tanf(10.0 * M_PI / 3 / tenth_acuity);
		int direction = rng() % 4;
		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(posTgt, LandoltCUp, cv::Size(size, size));
			break;
		case 1:
			proj.projectImageOnObject(posTgt, LandoltCDown, cv::Size(size, size));
			break;
		case 2:
			proj.projectImageOnObject(posTgt, LandoltCRight, cv::Size(size, size));
			break;
		case 3:
			proj.projectImageOnObject(posTgt, LandoltCLeft, cv::Size(size, size));
			break;
		default:
			proj.projectImageOnObject(posTgt, LandoltO, cv::Size(size, size));
			break;
		}
		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 65361:
			answer = 3;
			break;
		case 65362:
			answer = 0;
			break;
		case 65363:
			answer = 2;
			break;
		case 65364:
			answer = 1;
			break;
		default:
			break;
		}
		ofsD2 << tenth_acuity / 10.0 << direction << "," << answer << std::endl;
		try_count++;
		if (answer != direction)
		{
			count_failed++;
		}
		if (try_count == numOptoPerLine)
		{
			if (failed)
			{
				finished = true;
			}
			if (count_failed >= numFailedCriteria)
			{
				failed = true;
			}
			count_failed = 0;
			tenth_acuity--;
		}
	}
	ofsD2.close();
	odcs.Close();
	std::cout << "All tests finished." << std::endl;
	return 0;
}