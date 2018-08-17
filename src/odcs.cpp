#include <thread>
#include <vector>
#include <Eigen\Dense>
#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>
#define NOMINMAX
#include "odcs.hpp"
#include "arfModel.hpp"
#include "kalmanFilter.hpp"

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
		ods.DeterminePositionByDepthWithROI(*itr);
		Eigen::VectorXf duties = ocs.FindDutyQP((*itr)) * objPtrs.size();
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
	/*
	if (!thread_control.joinable())
	{
	thread_control.join();
	}
	*/
	
	ocs.Close();
}

void odcs::DetermineStateKF(FloatingObjectPtr objPtr, Eigen::VectorXf amplitudes)
{
	Eigen::Vector3f observe = ods.GetPositionByDepthWithROI(objPtr); //observe states of the object
	float dt = (timeGetTime() - objPtr->lastDeterminationTime) / 1000.0;
	Eigen::MatrixXf A(6, 6);
	A << Eigen::MatrixXf::Identity(3, 3), dt * Eigen::MatrixXf::Identity(),
		Eigen::MatrixXf::Zero(3, 3), Eigen::MatrixXf::Identity(3, 3);
	Eigen::MatrixXf B(6, ocs.positionAUTD.cols());
	Eigen::MatrixXf posRel = objPtr->getPosition().replicate(1, ocs.positionAUTD.cols()) - ocs.positionAUTD;
	Eigen::MatrixXf F = arfModel::arf(posRel) / (objPtr->mass + objPtr->additionalMass);
	B << Eigen::MatrixXf::Zero(3, ocs.positionAUTD.cols()), F;
	Eigen::VectorXf g(6); g << 0, 0, 0, 0, 0, objPtr->additionalMass * 9.806 / objPtr->totalMass;
	Eigen::VectorXf state;
	Eigen::MatrixXf P;
	Eigen::MatrixXf C(3, 6);
	Eigen::MatrixXf D(6, 6);
	Eigen::MatrixXf W(6, 6); // parameters of environment
	Eigen::MatrixXf V(3, 3); // parameters of kinect
	estimateStateKF(state, P, amplitudes, observe, A, B, g, C, D, W, V);
	//update states of object
}