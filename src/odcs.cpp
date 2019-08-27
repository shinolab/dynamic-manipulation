#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
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
	if (odsPtr == nullptr)
	{
		odsPtr = std::make_shared<ods>();
		odsPtr->Initialize();
	}
	if (ocsPtr == nullptr)
	{
		ocsPtr = std::make_shared<ocs>();
		ocsPtr->Initialize();
	}
}

std::shared_ptr<ods> odcs::Sensor() {
	return odsPtr;
}

std::shared_ptr<ocs> odcs::Controller() {
	return ocsPtr;
}

int odcs::AddObject(Eigen::Vector3f const &targetPosition)
{
	odcs::RegisterObject(FloatingObjectPtr(new FloatingObject(targetPosition)));
	return objPtrs.size();
}

void odcs::RegisterObject(FloatingObjectPtr objPtr)
{
	objPtrs.push_back(objPtr);
	ocsPtr->RegisterObject(objPtr);
}

const FloatingObjectPtr odcs::GetFloatingObject(int i)
{
	return ((i < objPtrs.size()) ? objPtrs[i] : nullptr);		
}

void odcs::ControlLoop(std::vector<FloatingObjectPtr> &objPtrs, int loopPeriod = 33)
{
	DWORD loopInit = timeGetTime();
	//Determination
	for (auto itr = objPtrs.begin(); itr != objPtrs.end(); itr++)
	{
		//----------Observation----------
		Eigen::Vector3f posObserved;
		bool succeeded = odsPtr->GetPositionByDepth((*itr), posObserved, true);
		DWORD observationTime = timeGetTime();
		if (succeeded && odsPtr->isInsideWorkSpace(posObserved))
		{
			//----------Determination----------
			(*itr)->updateStates(observationTime, posObserved);
			(*itr)->SetTrackingStatus(true);
		}
		else if (observationTime - (*itr)->lastDeterminationTime > 1000)
		{
			(*itr)->SetTrackingStatus(false);
		}
	}
	auto gains = ocsPtr->CreateBalanceGainMulti(objPtrs);
	//Update gain
	ocsPtr->_autd.ResetLateralGain();
	ocsPtr->_autd.AppendLateralGain(gains);
	ocsPtr->_autd.StartLateralModulation(100);

	int waitTime = loopPeriod - (timeGetTime() - loopInit);
	timeBeginPeriod(1);
	Sleep(std::max(waitTime, 0));
	timeEndPeriod(1);
}

bool odcs::isRunning() {
	std::shared_lock<std::shared_mutex> lk(mtxRunning);
	return flagRunning;
}

void odcs::StartControl()
{
	{
		std::lock_guard<std::shared_mutex> lock(mtxRunning);
		flagRunning = true;
	}
	thread_control = std::thread([this](){
		while (isRunning())
		{
			this->ControlLoop(objPtrs);
		}
	});
}

void odcs::Close()
{
	{
		std::lock_guard<std::shared_mutex> lk(mtxRunning);
		flagRunning = false;
	}
	if (thread_control.joinable()){
		thread_control.join();
	}
	ocsPtr->Close();
}

int odcs::AddDevice(Eigen::Vector3f const &position, Eigen::Vector3f const &eulerAngles) {
	return ocsPtr->AddDevice(position, eulerAngles);
}

void odcs::DetermineStateKF(FloatingObjectPtr objPtr, const Eigen::Vector3f &observe, const DWORD observationTime)
{
	float dt = (observationTime - objPtr->lastDeterminationTime) / 1000.0;
	//-----Construct System Matrix
	Eigen::MatrixXf A(6, 6);
	A << Eigen::Matrix3f::Identity(), dt * Eigen::Matrix3f::Identity(),
		Eigen::Matrix3f::Zero(), dt * Eigen::Matrix3f::Identity();
	Eigen::MatrixXf B(6, ocsPtr->positionsAUTD.cols());
	Eigen::MatrixXf posRel = objPtr->getPosition().replicate(1, ocsPtr->positionsAUTD.cols()) - ocsPtr->CentersAUTD();
	B << Eigen::MatrixXf::Zero(3, ocsPtr->positionsAUTD.cols()),
		ocsPtr->arfModelPtr->arf(posRel, ocsPtr->eulerAnglesAUTD) / objPtr->totalMass() * dt;
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

