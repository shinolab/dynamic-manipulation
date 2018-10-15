#define NOMINMAX
#include "projector.hpp"
#include "odcs.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <random>
#include <thread>

int main()
{
	std::string name;

	std::cout << "Please enter the participant's name : ";
	std::getline(std::cin, name);
	const int numOptoPerLine = 5;
	const int numFailedCriteria = 3;
	const int tenth_acuity_max = 29; //maximum 15@1000mm, 29 @ 2000mm
	const int tenth_acuity_min = 7;
	const int distance = 2000;
	const int longDistance = 1000;
	//Step 0: initialization
	std::cout << "Press enter key to capture the background including the tripod." << std::endl;
	std::getline(std::cin, std::string());
	odcs odcs;
	odcs.Initialize();
	FloatingObjectPtr objPtr = FloatingObject::Create(Eigen::Vector3f(0, 0, 1300));
	std::cout << "Press enter key to determine the default position of the balloon." << std::endl;
	std::getline(std::cin, std::string());
	//Step 1: Determine Position on a tripod.
	Eigen::Vector3f posTgt;
	while (1)
	{
		bool initSuccess = odcs.ods.GetPositionByDepth(objPtr, posTgt, false);
		if (initSuccess)
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
	cv::Mat LandoltO = cv::imread("img/LandoltC.bmp");
	Eigen::Vector3f projectionPoint = odcs.ods.getAffineKinect2Global().inverse() * posTgt;
	std::cout << "posTgt : " << posTgt.transpose() << std::endl;
	std::cout << "projection point : " << projectionPoint.transpose() << std::endl;

	bool failed = false;
	bool finished = false;
	unsigned int count_failed = 0;
	int try_count = 0;
	int tenth_acuity = tenth_acuity_min;

	//static image test
	std::ofstream ofsSUp(name + "_static_test_up.log");
	ofsSUp << "acuity, true direction, answer" << std::endl;

	while (!finished && !(tenth_acuity > tenth_acuity_max))
	{
		//white screen is displayed at the interval
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * distance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		std::cout << "acuity: " << tenth_acuity / 10.0 << ", try_count: " << try_count << ", count_failed: " << count_failed << ", failed: " << failed <<", finished: " << finished <<std::endl;
		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(projectionPoint, LandoltCUp, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 1:
			proj.projectImageOnObject(projectionPoint, LandoltCDown, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 2:
			proj.projectImageOnObject(projectionPoint, LandoltCRight, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 3:
			proj.projectImageOnObject(projectionPoint, LandoltCLeft, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		default:
			proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		}
		cv::waitKey(1000);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 'z':
			answer = 3;
			break;
		case 's':
			answer = 0;
			break;
		case 'c':
			answer = 2;
			break;
		case 'x':
			answer = 1;
			break;
		default:
			break;
		}
		ofsSUp << tenth_acuity / 10.0 << ", " << direction << "," << answer << std::endl;
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
			try_count = 0;
			tenth_acuity++; 
		}
	}
	ofsSUp.close();

	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	cv::waitKey(10); // time to display empty image
	std::cout << "Upward Test Ended.\n Press enter key to begin downward test(2)." << std::endl;

	bool succeeded = false;
	finished = false;
	int count_succeeded = 0;
	try_count = 0;
	tenth_acuity = tenth_acuity_max;
	std::ofstream ofsSDown(name + "_static_test_down.log");
	ofsSDown << "acuity, true direction, answer" << std::endl;

	while (!finished && !(tenth_acuity < tenth_acuity_min))
	{
		//white screen is displayed at the interval
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * distance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		std::cout << "acuity: " << tenth_acuity / 10.0 << ", try_count: " << try_count << ", count_succeeded: " << count_succeeded << ", succeeded: " << succeeded << ", finished: " << finished << std::endl;
		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(projectionPoint, LandoltCUp, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 1:
			proj.projectImageOnObject(projectionPoint, LandoltCDown, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 2:
			proj.projectImageOnObject(projectionPoint, LandoltCRight, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 3:
			proj.projectImageOnObject(projectionPoint, LandoltCLeft, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		default:
			proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		}
		cv::waitKey(1000);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), 0);

		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 'z':
			answer = 3;
			break;
		case 's':
			answer = 0;
			break;
		case 'c':
			answer = 2;
			break;
		case 'x':
			answer = 1;
			break;
		default:
			break;
		}
		ofsSDown << tenth_acuity / 10.0 << ", " << direction << "," << answer << std::endl;
		try_count++;
		if (answer == direction)
		{
			count_succeeded++;
		}
		if (try_count == numOptoPerLine)
		{
			if (succeeded)
			{
				finished = true;
			}
			if (count_succeeded >= numOptoPerLine - numFailedCriteria + 1)
			{
				succeeded = true;
			}
			count_succeeded = 0;
			try_count = 0;
			tenth_acuity--;
		}
	}
	ofsSDown.close();

	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	cv::waitKey(10); // time to display empty image
	std::cout << "Static Test Ended.\n Press enter key to begin static test(2)." << std::endl;
	std::getline(std::cin, std::string());
	
	failed = false;
	finished = false;
	count_failed = 0;
	try_count = 0;
	tenth_acuity = tenth_acuity_min;
	std::ofstream ofsS2(name + "_static_test2.log");
	ofsS2 << "acuity, true direction, answer" << std::endl;

	while (!finished && !(tenth_acuity > tenth_acuity_max))
	{
		//white screen is displayed at the interval
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * distance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		std::cout << "acuity: " << tenth_acuity / 10.0 << ", try_count: " << try_count << ", count_failed: " << count_failed << ", failed: " << failed << ", finished: " << finished << std::endl;
		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(projectionPoint, LandoltCUp, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 1:
			proj.projectImageOnObject(projectionPoint, LandoltCDown, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 2:
			proj.projectImageOnObject(projectionPoint, LandoltCRight, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 3:
			proj.projectImageOnObject(projectionPoint, LandoltCLeft, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		default:
			proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		}
		cv::waitKey(1000);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), 0);

		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 'z':
			answer = 3;
			break;
		case 's':
			answer = 0;
			break;
		case 'c':
			answer = 2;
			break;
		case 'x':
			answer = 1;
			break;
		default:
			break;
		}
		ofsS2 << tenth_acuity / 10.0 << ", " << direction << "," << answer << std::endl;
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
			try_count = 0;
			tenth_acuity++;
		}
	}
	ofsS2.close();

	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	cv::waitKey(10); // time to display empty image
	std::cout << "Upward Test Ended.\n Press enter key to begin downward test(2)." << std::endl;

	succeeded = false;
	finished = false;
	count_succeeded = 0;
	try_count = 0;
	tenth_acuity = tenth_acuity_max;
	std::ofstream ofsSDown2(name + "_static_test_down2.log");
	ofsSDown << "acuity, true direction, answer" << std::endl;

	while (!finished && !(tenth_acuity < tenth_acuity_min))
	{
		//white screen is displayed at the interval
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * distance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		std::cout << "acuity: " << tenth_acuity / 10.0 << ", try_count: " << try_count << ", count_succeeded: " << count_succeeded << ", succeeded: " << succeeded << ", finished: " << finished << std::endl;
		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(projectionPoint, LandoltCUp, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 1:
			proj.projectImageOnObject(projectionPoint, LandoltCDown, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 2:
			proj.projectImageOnObject(projectionPoint, LandoltCRight, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 3:
			proj.projectImageOnObject(projectionPoint, LandoltCLeft, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		default:
			proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		}
		cv::waitKey(1000);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), 0);

		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 'z':
			answer = 3;
			break;
		case 's':
			answer = 0;
			break;
		case 'c':
			answer = 2;
			break;
		case 'x':
			answer = 1;
			break;
		default:
			break;
		}
		ofsSDown2 << tenth_acuity / 10.0 << ", " << direction << "," << answer << std::endl;
		try_count++;
		if (answer == direction)
		{
			count_succeeded++;
		}
		if (try_count == numOptoPerLine)
		{
			if (succeeded)
			{
				finished = true;
			}
			if (count_succeeded >= numOptoPerLine - numFailedCriteria + 1)
			{
				succeeded = true;
			}
			count_succeeded = 0;
			try_count = 0;
			tenth_acuity--;
		}
	}
	ofsSDown2.close();

	proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(50, 50), cv::Scalar::all(0), -100);
	cv::waitKey(10); // time to display empty image
	std::cout << "Static Test Ended.\n Please remove the tripod. Then, press enter key to begin dynamic control." << std::endl;
	std::getline(std::cin, std::string());

	odcs.RegisterObject(objPtr);
	odcs.StartControl();

	std::cout << "Wait until the screen is stabilized. Then, press enter key to proceed to dynamic acuity test." << std::endl;
	std::getline(std::cin, std::string());

	bool log_stop = false;
	std::thread thread_log([&objPtr, &log_stop, &name](){
		std::ofstream ofsObj(name + "manipulation_log.csv");
		DWORD initTime = timeGetTime();
		ofsObj << "time[ms], x[mm], y[mm], z[mm], xTgt[mm], yTgt[mm], zTgt[mm]" << std::endl;
		while (!log_stop)
		{
			Eigen::Vector3f pos = objPtr->getPosition();
			Eigen::Vector3f posTarget = objPtr->getPositionTarget();
			ofsObj << timeGetTime() - initTime << ", " << pos.x() << ", " << pos.y() << "," << pos.z() << ", "
				<< posTarget.x() << ", " << posTarget.y() << ", " << posTarget.z() << std::endl;
			Sleep(30);
		}
	}
	);
	
	std::ofstream ofsD1(name + "_dynamic_test1.log");
	ofsD1 << "acuity, true direction, answer" << std::endl;
	failed = false;
	finished = false;
	count_failed = 0;
	try_count = 0;
	tenth_acuity = tenth_acuity_min;
	while (!finished && !(tenth_acuity > tenth_acuity_max))
	{
		proj.projectImageOnObject(posTgt, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * distance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		std::cout << "acuity: " << tenth_acuity / 10.0 << ", try_count: " << try_count << ", count_failed: " << count_failed << ", failed: " << failed << ", finished: " << finished << std::endl;

		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(projectionPoint, LandoltCUp, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 1:
			proj.projectImageOnObject(projectionPoint, LandoltCDown, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 2:
			proj.projectImageOnObject(projectionPoint, LandoltCRight, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 3:
			proj.projectImageOnObject(projectionPoint, LandoltCLeft, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		default:
			proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		}
		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 'z':
			answer = 3;
			break;
		case 's':
			answer = 0;
			break;
		case 'c':
			answer = 2;
			break;
		case 'x':
			answer = 1;
			break;
		default:
			break;
		}
		ofsD1 << tenth_acuity / 10.0 << ", " << direction << ", " << answer << std::endl;
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
			try_count = 0;
			count_failed = 0;
			tenth_acuity++;
		}
	}
	ofsD1.close();

	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	cv::waitKey(10); // time to display empty image
	std::cout << "Upward Test Ended.\n Press enter key to begin downward test(2)." << std::endl;

	succeeded = false;
	finished = false;
	count_succeeded = 0;
	try_count = 0;
	tenth_acuity = tenth_acuity_max;
	std::ofstream ofsDDown(name + "_dynamic_test_down.log");
	ofsDDown << "acuity, true direction, answer" << std::endl;

	while (!finished && !(tenth_acuity < tenth_acuity_min))
	{
		//white screen is displayed at the interval
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * distance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		std::cout << "acuity: " << tenth_acuity / 10.0 << ", try_count: " << try_count << ", count_succeeded: " << count_succeeded << ", succeeded: " << succeeded << ", finished: " << finished << std::endl;
		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(projectionPoint, LandoltCUp, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 1:
			proj.projectImageOnObject(projectionPoint, LandoltCDown, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 2:
			proj.projectImageOnObject(projectionPoint, LandoltCRight, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 3:
			proj.projectImageOnObject(projectionPoint, LandoltCLeft, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		default:
			proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		}
		cv::waitKey(1000);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), 0);

		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 'z':
			answer = 3;
			break;
		case 's':
			answer = 0;
			break;
		case 'c':
			answer = 2;
			break;
		case 'x':
			answer = 1;
			break;
		default:
			break;
		}
		ofsDDown << tenth_acuity / 10.0 << ", " << direction << "," << answer << std::endl;
		try_count++;
		if (answer == direction)
		{
			count_succeeded++;
		}
		if (try_count == numOptoPerLine)
		{
			if (succeeded)
			{
				finished = true;
			}
			if (count_succeeded >= numOptoPerLine - numFailedCriteria + 1)
			{
				succeeded = true;
			}
			count_succeeded = 0;
			try_count = 0;
			tenth_acuity--;
		}
	}
	ofsDDown.close();

	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	std::cout << "Dynamic Test (1) Ended.\n Press enter key to begin dynamic acuity test (2)." << std::endl;
	cv::waitKey(10); // time to display empty image
	std::getline(std::cin, std::string());

	std::ofstream ofsD2(name + "_dynamic_test2.log");
	ofsD2 << "acuity, true direction, answer" << std::endl;
	failed = false;
	finished = false;
	count_failed = 0;
	try_count = 0;
	tenth_acuity = tenth_acuity_min;
	while (!finished && !(tenth_acuity > tenth_acuity_max))
	{
		proj.projectImageOnObject(posTgt, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * distance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		std::cout << "acuity: " << tenth_acuity / 10.0 << ", try_count: " << try_count << ", count_failed: " << count_failed << ", failed: " << failed << ", finished: " << finished << std::endl;

		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(projectionPoint, LandoltCUp, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 1:
			proj.projectImageOnObject(projectionPoint, LandoltCDown, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 2:
			proj.projectImageOnObject(projectionPoint, LandoltCRight, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 3:
			proj.projectImageOnObject(projectionPoint, LandoltCLeft, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		default:
			proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		}
		cv::waitKey(1000);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), 0);

		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 'z':
			answer = 3;
			break;
		case 's':
			answer = 0;
			break;
		case 'c':
			answer = 2;
			break;
		case 'x':
			answer = 1;
			break;
		default:
			break;
		}
		ofsD2 << tenth_acuity / 10.0 << ", " << direction << "," << answer << std::endl;
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
			try_count = 0;
			count_failed = 0;
			tenth_acuity++;
		}
	}
	ofsD2.close();

	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	cv::waitKey(10); // time to display empty image
	std::cout << "Upward Test Ended.\n Press enter key to begin downward test(2)." << std::endl;

	succeeded = false;
	finished = false;
	count_succeeded = 0;
	try_count = 0;
	tenth_acuity = tenth_acuity_max;
	std::ofstream ofsD2Down(name + "_dynamic_test_down2.log");
	ofsD2Down << "acuity, true direction, answer" << std::endl;

	while (!finished && !(tenth_acuity < tenth_acuity_min))
	{
		//white screen is displayed at the interval
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * distance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		std::cout << "acuity: " << tenth_acuity / 10.0 << ", try_count: " << try_count << ", count_succeeded: " << count_succeeded << ", succeeded: " << succeeded << ", finished: " << finished << std::endl;
		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(projectionPoint, LandoltCUp, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 1:
			proj.projectImageOnObject(projectionPoint, LandoltCDown, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 2:
			proj.projectImageOnObject(projectionPoint, LandoltCRight, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 3:
			proj.projectImageOnObject(projectionPoint, LandoltCLeft, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		default:
			proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		}
		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 'z':
			answer = 3;
			break;
		case 's':
			answer = 0;
			break;
		case 'c':
			answer = 2;
			break;
		case 'x':
			answer = 1;
			break;
		default:
			break;
		}
		cv::waitKey(1000);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), 0);

		ofsD2Down << tenth_acuity / 10.0 << ", " << direction << "," << answer << std::endl;
		try_count++;
		if (answer == direction)
		{
			count_succeeded++;
		}
		if (try_count == numOptoPerLine)
		{
			if (succeeded)
			{
				finished = true;
			}
			if (count_succeeded >= numOptoPerLine - numFailedCriteria + 1)
			{
				succeeded = true;
			}
			count_succeeded = 0;
			try_count = 0;
			tenth_acuity--;
		}
	}
	ofsD2Down.close();

	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	std::cout << "Dynamic Test (2) Ended.\n Press enter key to begin dynamic acuity test (3)." << std::endl;
	cv::waitKey(10); // time to display empty image
	std::getline(std::cin, std::string());

	std::ofstream ofsD3(name + "_dynamic_test3.log");
	ofsD3 << "acuity, true direction, answer" << std::endl;
	failed = false;
	finished = false;
	count_failed = 0;
	try_count = 0;
	tenth_acuity = tenth_acuity_min;
	while (!finished && !(tenth_acuity > tenth_acuity_max))
	{
		proj.projectImageOnObject(posTgt, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * longDistance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		std::cout << "acuity: " << tenth_acuity / 10.0 << ", try_count: " << try_count << ", count_failed: " << count_failed << ", failed: " << failed << ", finished: " << finished << std::endl;

		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(projectionPoint, LandoltCUp, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 1:
			proj.projectImageOnObject(projectionPoint, LandoltCDown, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 2:
			proj.projectImageOnObject(projectionPoint, LandoltCRight, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 3:
			proj.projectImageOnObject(projectionPoint, LandoltCLeft, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		default:
			proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		}
		cv::waitKey(1000);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), 0);

		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 'z':
			answer = 3;
			break;
		case 's':
			answer = 0;
			break;
		case 'c':
			answer = 2;
			break;
		case 'x':
			answer = 1;
			break;
		default:
			break;
		}
		ofsD3 << tenth_acuity / 10.0 << ", " << direction << "," << answer << std::endl;
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
			try_count = 0;
			count_failed = 0;
			tenth_acuity++;
		}
	}
	ofsD3.close();

	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	cv::waitKey(10); // time to display empty image
	std::cout << "Upward Test Ended.\n Press enter key to begin downward test(2)." << std::endl;

	succeeded = false;
	finished = false;
	count_succeeded = 0;
	try_count = 0;
	tenth_acuity = tenth_acuity_max;
	std::ofstream ofsD3Down(name + "_dynamic_test_down3.log");
	ofsD3Down << "acuity, true direction, answer" << std::endl;

	while (!finished && !(tenth_acuity < tenth_acuity_min))
	{
		//white screen is displayed at the interval
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * longDistance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		std::cout << "acuity: " << tenth_acuity / 10.0 << ", try_count: " << try_count << ", count_succeeded: " << count_succeeded << ", succeeded: " << succeeded << ", finished: " << finished << std::endl;
		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(projectionPoint, LandoltCUp, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 1:
			proj.projectImageOnObject(projectionPoint, LandoltCDown, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 2:
			proj.projectImageOnObject(projectionPoint, LandoltCRight, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 3:
			proj.projectImageOnObject(projectionPoint, LandoltCLeft, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		default:
			proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		}
		cv::waitKey(1000);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), 0);

		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 'z':
			answer = 3;
			break;
		case 's':
			answer = 0;
			break;
		case 'c':
			answer = 2;
			break;
		case 'x':
			answer = 1;
			break;
		default:
			break;
		}
		ofsDDown << tenth_acuity / 10.0 << ", " << direction << "," << answer << std::endl;
		try_count++;
		if (answer == direction)
		{
			count_succeeded++;
		}
		if (try_count == numOptoPerLine)
		{
			if (succeeded)
			{
				finished = true;
			}
			if (count_succeeded >= numOptoPerLine - numFailedCriteria + 1)
			{
				succeeded = true;
			}
			count_succeeded = 0;
			try_count = 0;
			tenth_acuity--;
		}
	}
	ofsD3Down.close();

	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	std::cout << "Dynamic Test (3) Ended.\n Press enter key to begin dynamic acuity test (4)." << std::endl;
	cv::waitKey(10); // time to display empty image
	std::getline(std::cin, std::string());

	std::ofstream ofsD4(name + "_dynamic_test4.log");
	ofsD4 << "acuity, true direction, answer" << std::endl;
	failed = false;
	finished = false;
	count_failed = 0;
	try_count = 0;
	tenth_acuity = tenth_acuity_min;
	while (!finished && !(tenth_acuity > tenth_acuity_max))
	{
		proj.projectImageOnObject(posTgt, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * longDistance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		std::cout << "acuity: " << tenth_acuity / 10.0 << ", try_count: " << try_count << ", count_failed: " << count_failed << ", failed: " << failed << ", finished: " << finished << std::endl;

		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(projectionPoint, LandoltCUp, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 1:
			proj.projectImageOnObject(projectionPoint, LandoltCDown, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 2:
			proj.projectImageOnObject(projectionPoint, LandoltCRight, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 3:
			proj.projectImageOnObject(projectionPoint, LandoltCLeft, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		default:
			proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		}
		cv::waitKey(1000);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), 0);

		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 'z':
			answer = 3;
			break;
		case 's':
			answer = 0;
			break;
		case 'c':
			answer = 2;
			break;
		case 'x':
			answer = 1;
			break;
		default:
			break;
		}
		ofsD4 << tenth_acuity / 10.0 << ", " << direction << "," << answer << std::endl;
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
			try_count = 0;
			count_failed = 0;
			tenth_acuity++;
		}
	}
	ofsD4.close();

	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	cv::waitKey(10); // time to display empty image
	std::cout << "Upward Test Ended.\n Press enter key to begin downward test(2)." << std::endl;

	succeeded = false;
	finished = false;
	count_succeeded = 0;
	try_count = 0;
	tenth_acuity = tenth_acuity_max;
	std::ofstream ofsD4Down(name + "_dynamic_test_down4.log");
	ofsD4Down << "acuity, true direction, answer" << std::endl;

	while (!finished && !(tenth_acuity < tenth_acuity_min))
	{
		//white screen is displayed at the interval
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * longDistance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		std::cout << "acuity: " << tenth_acuity / 10.0 << ", try_count: " << try_count << ", count_succeeded: " << count_succeeded << ", succeeded: " << succeeded << ", finished: " << finished << std::endl;
		switch (direction)
		{
		case 0:
			proj.projectImageOnObject(projectionPoint, LandoltCUp, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 1:
			proj.projectImageOnObject(projectionPoint, LandoltCDown, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 2:
			proj.projectImageOnObject(projectionPoint, LandoltCRight, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		case 3:
			proj.projectImageOnObject(projectionPoint, LandoltCLeft, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		default:
			proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(size, size), cv::Scalar::all(0), -100);
			break;
		}
		cv::waitKey(1000);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), 0);

		auto key = cv::waitKey(0);
		int answer = -1;
		switch (key)
		{
		case 'z':
			answer = 3;
			break;
		case 's':
			answer = 0;
			break;
		case 'c':
			answer = 2;
			break;
		case 'x':
			answer = 1;
			break;
		default:
			break;
		}
		ofsD4Down << tenth_acuity / 10.0 << ", " << direction << "," << answer << std::endl;
		try_count++;
		if (answer == direction)
		{
			count_succeeded++;
		}
		if (try_count == numOptoPerLine)
		{
			if (succeeded)
			{
				finished = true;
			}
			if (count_succeeded >= numOptoPerLine - numFailedCriteria + 1)
			{
				succeeded = true;
			}
			count_succeeded = 0;
			try_count = 0;
			tenth_acuity--;
		}
	}
	ofsD4Down.close();

	log_stop = true;
	thread_log.join();
	odcs.Close();

	std::cout << "All tests finished." << std::endl;
	return 0;
}