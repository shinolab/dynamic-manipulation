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

int odcs::AddObject(Eigen::Vector3f targetPosition)
{
	odcs::RegisterObject(FloatingObjectPtr(new FloatingObject(targetPosition)));
	return objPtrs.size();
}

void odcs::RegisterObject(FloatingObjectPtr objPtr)
{
	objPtrs.push_back(objPtr);
	ocs.RegisterObject(objPtr);
}

const FloatingObjectPtr odcs::GetFloatingObject(int i)
{
	return ((i < objPtrs.size()) ? objPtrs[i] : nullptr);		
}

void odcs::ControlLoop(std::vector<FloatingObjectPtr> &objPtrs, int loopPeriod = 30)
{
	DWORD timeInit = timeGetTime();
	int periodPerObject = loopPeriod / objPtrs.size();
	for (auto itr = objPtrs.begin(); itr != objPtrs.end(); itr++)
	{
		ods.DeterminePositionByDepth(*itr, true);
		Eigen::VectorXf amplitudes = ocs.FindDutyQP((*itr)) * objPtrs.size();
		Eigen::VectorXi duties = (510 / M_PI * amplitudes.array().sqrt().asin().max(0).min(255)).matrix().cast<int>();
		ocs.DirectSemiPlaneWave((*itr), duties);	
	}
	int waitTime = loopPeriod - (timeGetTime() - timeInit);
	Sleep(std::max(waitTime, 0));
}

void odcs::StartControl()
{
	thread_control = std::thread([this](){
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
	
	if (thread_control.joinable())
	{
	thread_control.join();
	}
	ocs.Close();
}

void odcs::DetermineStateKF(FloatingObjectPtr objPtr, const Eigen::Vector3f observe, const DWORD observationTime)
{
	float dt = (observationTime - objPtr->lastDeterminationTime) / 1000.0;
	//-----Construct System Matrix
	Eigen::MatrixXf A(6, 6);
	A << Eigen::Matrix3f::Identity(), dt * Eigen::Matrix3f::Identity(),
		Eigen::Matrix3f::Zero(), dt * Eigen::Matrix3f::Identity();
	Eigen::MatrixXf B(6, ocs.positionAUTD.cols());
	Eigen::MatrixXf posRel = objPtr->getPosition().replicate(1, ocs.positionAUTD.cols()) - ocs.centerAUTD;
	B << Eigen::MatrixXf::Zero(3, ocs.positionAUTD.cols()),
		ocs.arfModelPtr->arf(posRel, ocs.eulerAnglesAUTD) / objPtr->totalMass() * dt;
	Eigen::VectorXf g(6); g << Eigen::Vector3f::Zero(), dt * objPtr->additionalMass * Eigen::Vector3f(0, 0, -9.801e3) / objPtr->totalMass() * dt;
	Eigen::MatrixXf C(3, 6); C << Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Zero();
	Eigen::MatrixXf D(6, 6); D.setIdentity();
	Eigen::VectorXf w(6); w << 10, 10, 10, 0.25 * Eigen::Vector3f::Ones() * dt / objPtr->totalMass();
	Eigen::MatrixXf covDisturbance(6, 6); covDisturbance = w.asDiagonal();
	Eigen::Matrix3f covNoise = 3 * Eigen::Matrix3f::Identity(); //assuming that errors in all directions are independent.
	//-----estimation
	Eigen::VectorXf state(6); state << objPtr->getPosition(), objPtr->getVelocity();
	Eigen::MatrixXf covError = objPtr->covError;
	estimateStateKF(state, covError, objPtr->getLatestInput(), observe, A, B, g, C, D, covDisturbance, covNoise);
	Eigen::Vector3f pos_next = state.head(3);
	Eigen::Vector3f vel_next = state.tail(3);
	objPtr->updateStates(observationTime, pos_next, vel_next);
	objPtr->covError = covError;
}

