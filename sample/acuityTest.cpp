#define NOMINMAX
#include "projector.hpp"
#include "odcs.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <numeric>
#include <random>
#include <thread>
#include <algorithm>

void projectCalibrationRing(projector &proj, const Eigen::Vector3f &projectionPoint)
{
	cv::Mat LandoltO = cv::imread("img/LandoltC.bmp");
	proj.projectImageOnObject(projectionPoint, LandoltO, cv::Size(10, 10), cv::Scalar::all(0), -100);
	cv::waitKey(100);
}



void runProjectionSequenceSimple(const std::string testName, projector &proj, const Eigen::Vector3f &projectionPoint, const int distance, bool ascending = true)
{
	const int tenth_acuity_max = 26; //maximum 14@1000mm, 29 @ 2000mm
	const int tenth_acuity_min = 3;
	const int displayTime = 2000; //[ms]

	bool finished = false;
	proj.RefleshScreen(); cv::waitKey(1);
	std::random_device rng;
	cv::Mat LandoltCUp = cv::imread("img/LandoltC_up.bmp");
	cv::Mat LandoltCDown = cv::imread("img/LandoltC_down.bmp");
	cv::Mat LandoltCRight = cv::imread("img/LandoltC_right.bmp");
	cv::Mat LandoltCLeft = cv::imread("img/LandoltC_left.bmp");
	cv::Mat LandoltO = cv::imread("img/LandoltC.bmp");

	int tenth_acuity = (ascending ? tenth_acuity_min : tenth_acuity_max);
	//static image test
	std::string postfix = (ascending ? "_ascend.csv" : "_descend.csv");
	std::ofstream ofs(testName + postfix);
	bool failed = false;
	int count = 0;
	while (!finished)
	{
		//white screen is displayed at the interval
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "press any key to proceed" << std::endl;
		cv::waitKey(0);
		float size = 5 * distance * tanf(10.0 * M_PI / 180 / 60 / tenth_acuity);
		int direction = rng() % 4;
		if (ascending) {
			std::cout << "acuity: " << tenth_acuity / 10.0 << ", failed: " << failed << ", finished: " << finished << std::endl;
		}
		else{
			std::cout << "acuity: " << tenth_acuity / 10.0 << ", succeeded: " << !failed << ", finished: " << finished << std::endl;
		}
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
		cv::waitKey(displayTime);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "please answer." << std::endl;
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
		case 'q':
			answer = -1;
			finished = true;
			break;
		case 'n':
			answer = -3;
			break;
		case 'b':
			answer = -2;
			tenth_acuity += (ascending ? -2 : 2);
			break;
		default:
			break;
		}
		failed = (answer != direction) ? true : false;
		ofs << tenth_acuity / 10.0 << ", " << direction << "," << answer << "," << failed << std::endl;
		count += false ? (ascending ? 1 : 0) : (ascending ? 0 : 1);
		tenth_acuity += (ascending ? 1 : -1);

	}

}

void runMethodOfLimits(const std::string testName, projector &proj, const Eigen::Vector3f &projectionPoint, const int distance, const int num = 6)
{
	std::vector<int> order(num);
	for (int i = 0; i < num; i++)
	{
		order[i] = i;
	}
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());
	std::shuffle(order.begin(), order.end(), engine);
	for (int i = 0; i < num; i++)
	{
		bool isAscending = (order[i] % 2 == 0);
		runProjectionSequenceSimple(testName + std::to_string(i), proj, projectionPoint, distance, isAscending);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		cv::waitKey(10);
	}
}

void runProjectionSequence(const std::string testName, projector &proj, const Eigen::Vector3f &projectionPoint, const int distance)
{
	proj.RefleshScreen(); cv::waitKey(1);
	const int numOptoPerLine = 10;
	const int numFailedCriteria = 7;
	const int tenth_acuity_max = 20; //maximum 14@1000mm, 29 @ 2000mm
	const int tenth_acuity_min = 5;
	const int displayTime = 2000; //[ms]
	std::random_device rng;
	cv::Mat LandoltCUp = cv::imread("img/LandoltC_up.bmp");
	cv::Mat LandoltCDown = cv::imread("img/LandoltC_down.bmp");
	cv::Mat LandoltCRight = cv::imread("img/LandoltC_right.bmp");
	cv::Mat LandoltCLeft = cv::imread("img/LandoltC_left.bmp");
	cv::Mat LandoltO = cv::imread("img/LandoltC.bmp");
	bool failed = false;
	bool finished = false;
	unsigned int count_failed = 0;
	int try_count = 0;
	int tenth_acuity = tenth_acuity_min;
	//static image test
	std::ofstream ofs(testName + "_up.log");
	ofs << "acuity, true direction, answer" << std::endl;
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
		cv::waitKey(displayTime);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		std::cout << "please answer." << std::endl;
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
		case 'q':
			finished = true;
			break;
		case 'n':
			try_count = numOptoPerLine;
			count_failed = -1;
			answer = -2;
			break;
		case 'b':
			try_count = numOptoPerLine;
			count_failed = -1;
			answer = -3;
			tenth_acuity -= 2;
			break;
		default:
			break;
		}
		ofs << tenth_acuity / 10.0 << ", " << direction << "," << answer << std::endl;
		try_count++;
		if (answer != direction)
		{
			count_failed++;
		}
		if (try_count >= numOptoPerLine)
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
	ofs.close();
	std::cout << "Upward seqence ended. Downward sequence begins now." << std::endl;

	bool succeeded = false;
	finished = false;
	int count_succeeded = 0;
	try_count = 0;
	tenth_acuity = tenth_acuity_max;
	std::ofstream ofsSDown(testName + "_down.log");
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
		cv::waitKey(displayTime);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), 0);
		std::cout << "please answer." << std::endl;
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
		case 'q':
			finished = true;
			break;
		case 'n':
			try_count = numOptoPerLine;
			count_succeeded = -1;
			answer = -2;
			break;
		case 'b':
			try_count = numOptoPerLine;
			count_succeeded = -1;
			answer = -3;
			tenth_acuity += 2;
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
		if (try_count >= numOptoPerLine)
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

}

int main()
{
	std::string name;
	std::cout << "Please enter the participant's name : ";
	std::getline(std::cin, name);
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
	Eigen::Vector3f projectionPoint = odcs.ods.getAffineKinect2Global().inverse() * posTgt;
	std::cout << "posTgt : " << posTgt.transpose() << std::endl;
	std::cout << "projection point : " << projectionPoint.transpose() << std::endl;

	projector proj("projecor");
	projectCalibrationRing(proj, projectionPoint);

	std::cout << "Which test do you conduct first ? [s:static] / d:dynamic :";
	std::string which;
	std::getline(std::cin, which);
	if (which != "d")
	{

		std::cout << "Press enter key to start practice (static)." << std::endl;
		std::getline(std::cin, std::string());

		runProjectionSequenceSimple(name + "_static_practice", proj, projectionPoint, distance, true);
		runProjectionSequenceSimple(name + "_static_practice", proj, projectionPoint, distance, false);

		//runProjectionSequence(name + "_static_practice", proj, projectionPoint, distance);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		cv::waitKey(10);
		std::cout << "Practice test (static) Ended.\n Press enter key to begin static acuity test." << std::endl;
		std::getline(std::cin, std::string());

		runMethodOfLimits(name + "_static1_", proj, projectionPoint, distance);
		//runProjectionSequence(name + "static1", proj, projectionPoint, distance);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		cv::waitKey(10);

		/*
		std::cout << "Static Test (1) Ended.\n Press enter key to begin static acuity test (2)." << std::endl;
		std::getline(std::cin, std::string());
		runMethodOfLimits(name + "_static2_", proj, projectionPoint, distance);
		//runProjectionSequence(name + "static2", proj, projectionPoint, distance);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		cv::waitKey(10);

		*/

		std::cout << "Static Test Ended.\n Please remove the tripod. Then, press enter key to begin dynamic control." << std::endl;

	}
	else
	{
		std::cout << "Please remove the tripod. Then, press enter key to begin dynamic control." << std::endl;
	}
	std::getline(std::cin, std::string());

	odcs.RegisterObject(objPtr);
	odcs.StartControl();
	std::cout << "Wait until the screen is stabilized. Then, press enter key to proceed to dynamic acuity test." << std::endl;
	projectCalibrationRing(proj, projectionPoint);
	cv::waitKey(10);
	std::getline(std::cin, std::string());

	bool log_stop = false;
	std::thread thread_log([&objPtr, &log_stop, &name](){
		std::ofstream ofsObj(name + "_manipulation_log.csv");
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

	runProjectionSequenceSimple(name + "_dynamic_practice", proj, projectionPoint, distance, true);
	runProjectionSequenceSimple(name + "_dynamic_practice", proj, projectionPoint, distance, false);

	//runProjectionSequence(name + "practice_dynamic", proj, projectionPoint, distance);
	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	cv::waitKey(10);
	std::cout << "Practice Test (dynamic) Ended.\n Press enter key to begin dynamic acuity test." << std::endl;
	std::getline(std::cin, std::string());

	runMethodOfLimits(name + "_dynamic1_", proj, projectionPoint, distance);
	//runProjectionSequence(name + "dynamic1", proj, projectionPoint, distance);
	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	cv::waitKey(10);
	std::cout << "Dynamic Test (1) Ended.\n Press enter key to proceed." << std::endl;
	std::getline(std::cin, std::string());
	
	/*
	std::cout << "Dynamic Test (1) Ended.\n Press enter key to begin dynamic acuity test (2)." << std::endl;
	std::getline(std::cin, std::string());

	runMethodOfLimits(name + "_dynamic2_", proj, projectionPoint, distance);

	//runProjectionSequence(name + "dynamic2", proj, projectionPoint, distance);
	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	cv::waitKey(10);
	std::cout << "Dynamic Test (2) Ended.\n Press enter key to begin dynamic acuity test (3)." << std::endl;
	std::getline(std::cin, std::string());

	runMethodOfLimits(name + "_dynamic3_", proj, projectionPoint, longDistance);
	//runProjectionSequence(name + "dynamic3", proj, projectionPoint, longDistance);
	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	cv::waitKey(10);
	std::cout << "Dynamic Test (3) Ended.\n Press enter key to begin dynamic acuity test (4)." << std::endl;
	std::getline(std::cin, std::string());

	runMethodOfLimits(name + "_dynamic4_", proj, projectionPoint, longDistance);
	//runProjectionSequence(name + "dynamic4", proj, projectionPoint, longDistance);
	proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
	cv::waitKey(10);
	std::cout << "Dynamic Test (4) Ended." << std::endl;
	std::getline(std::cin, std::string());
	*/
	if (which == "d")
	{
		log_stop = true;

		std::cout << "Dynamic Test (1) Ended.\n Place the balloon on a tripod. Then, press enter ket to proceed." << std::endl;
		std::getline(std::cin, std::string());
		std::cout << "Press enter key to start practice (static)." << std::endl;
		std::getline(std::cin, std::string());

		runProjectionSequenceSimple(name + "_static_practice", proj, projectionPoint, distance, true);
		runProjectionSequenceSimple(name + "_static_practice", proj, projectionPoint, distance, false);

		//runProjectionSequence(name + "_static_practice", proj, projectionPoint, distance);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		cv::waitKey(10);
		std::cout << "Practice test (static) Ended.\n Press enter key to begin static acuity test." << std::endl;
		std::getline(std::cin, std::string());

		runMethodOfLimits(name + "_static1_", proj, projectionPoint, distance);
		//runProjectionSequence(name + "static1", proj, projectionPoint, distance);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		cv::waitKey(10);

		/*
		std::cout << "Static Test (1) Ended.\n Press enter key to begin static acuity test (2)." << std::endl;
		std::getline(std::cin, std::string());
		runMethodOfLimits(name + "_static2_", proj, projectionPoint, distance);
		//runProjectionSequence(name + "static2", proj, projectionPoint, distance);
		proj.projectImageOnObject(projectionPoint, cv::Mat(100, 100, CV_8UC3, cv::Scalar::all(0)), cv::Size(100, 100), cv::Scalar::all(0), -100);
		cv::waitKey(10);

		*/

		std::cout << "Static Test Ended.\n Thank you very much." << std::endl;
	}

	log_stop = true;
	thread_log.join();
	std::cout << "log stopped." << std::endl;
	std::cout << "Press q on control window. Then, press enter key on the console window to end the test." << std::endl;
	std::getline(std::cin, std::string());
	odcs.Close();

	std::cout << "All tests finished." << std::endl;
	return 0;
}