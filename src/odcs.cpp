#include <thread>
#include <vector>
#include <Eigen\Dense>
#include "odcs.hpp"

#define _USE_MATH_DEFINES
#include<math.h>


void odcs::Initialize()
{
	ods.Initialize();
	ocs.Initialize();
}

void odcs::ControlLoop(std::vector<FloatingObjectPtr> objPtrs)
{
	for (auto itr = objPtrs.begin(); itr != objPtrs.end(); itr++)
	{
		ods.DeterminePositionByDepth(objPtrs);
		Eigen::VectorXf duties = ocs.FindDutyQP((*itr)) / objPtrs.size();
		Eigen::VectorXi amplitudes = (510 / M_PI * duties.array().sqrt().asin().max(0).min(255)).matrix().cast<int>();
		ocs.DirectSemiPlaneWave((*itr), amplitudes);
	}
}

void odcs::StartControl(std::vector<FloatingObjectPtr> &objPtrs)
{
	thread_control = std::thread([this, &objPtrs](){
		cv::imshow("controlwindow", cv::Mat::zeros(500, 500, CV_8UC1));
		Sleep(5);
		while (1)
		{
			this->ControlLoop(objPtrs);
			auto key = cv::waitKey(1);
			if (key == 'q')
			{
				cv::destroyAllWindows();
				break;
			}
		}
	});
}

void odcs::Close()
{
	if (!thread_control.joinable())
	{
		thread_control.join();
	}
	ocs.Close();
}