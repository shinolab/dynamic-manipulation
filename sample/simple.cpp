#include "odcs.hpp"
#include "kalmanFilter.hpp"
#include <Eigen\Geometry>
#include <iostream>
#include <fstream>
#include <vector>
#include <Windows.h>
#include <opencv2/core.hpp>
#include <opencv2/shape.hpp>

#pragma comment(lib, "winmm.lib")

int main()
{
	odcs odcs; odcs.Initialize();//ocs.SetGain(-1.6*Eigen::Vector3f::Ones(), -2.53 * Eigen::Vector3f::Ones(), -0.1*Eigen::Vector3f::Ones());
	FloatingObjectPtr objPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1085)));
	std::cout << "current position : " << objPtr->getPosition().transpose() << std::endl;
	Eigen::VectorXf duties = odcs.ocs.FindDutyQP(odcs.ocs.ComputePIDForce(objPtr), objPtr->getPosition());
	objPtr->lastDeterminationTime = timeGetTime();
	Sleep(10);
	std::cout << "hello" << std::endl;
	DWORD observationTime = timeGetTime();
	float dt = (observationTime - objPtr->lastDeterminationTime) / 1000.0;
	std::cout << "dt : " << dt << std::endl;
	Eigen::MatrixXf A(6, 6);
	A << Eigen::Matrix3f::Identity(), dt * Eigen::Matrix3f::Identity(),
		Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Identity();
	Eigen::MatrixXf B(6, odcs.ocs.positionAUTD.cols());
	std::cout << "A :\n" << A << std::endl;
	Eigen::MatrixXf posRel = objPtr->getPosition().replicate(1, odcs.ocs.positionAUTD.cols()) - odcs.ocs.centerAUTD;
	std::cout << "posRel : \n" << posRel << std::endl;
	std::cout << "ARF Model : \n" << odcs.ocs.arfModelPtr->arf(posRel, odcs.ocs.eulerAnglesAUTD) << std::endl;
	B << Eigen::MatrixXf::Zero(3, odcs.ocs.positionAUTD.cols()),
		odcs.ocs.arfModelPtr->arf(posRel, odcs.ocs.eulerAnglesAUTD) / objPtr->totalMass() * dt;
	std::cout << "B :\n" << B << std::endl;
	Eigen::VectorXf g(6); g << Eigen::Vector3f::Zero(), dt * objPtr->additionalMass * Eigen::Vector3f(0, 0, -9.801e3) / objPtr->totalMass() * dt;
	std::cout << "g : " << g.transpose() << std::endl;
	Eigen::VectorXf state(6); state << objPtr->getPosition(), objPtr->getVelocity();
	Eigen::MatrixXf C(3, 6); C << Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Zero();
	std::cout << "C : \n" << C << std::endl;
	Eigen::MatrixXf D(6, 6); D.setIdentity();
	std::cout << "D : \n" << D << std::endl;
	Eigen::VectorXf w(6); w << 10, 10, 10, 30, 30, 30; w *= dt;
	Eigen::MatrixXf W(6, 6); W = w.asDiagonal();
	Eigen::Matrix3f V = 0.003 * Eigen::Matrix3f::Identity(); //assuming that errors in all directions are independent.
	Eigen::MatrixXf covError = objPtr->covError;
	Eigen::Vector3f posObserved = objPtr->getPosition();
	std::cout << "W : \n" << W << std::endl;
	std::cout << "V : \n" << V << std::endl;
	std::cout << "state(before) : " << state.transpose() << std::endl;
	std::cout << "covError(before) : \n" << covError << std::endl;
	DWORD start = timeGetTime();
	for(int i = 0; i < 5; i++)
		estimateStateKF(state, covError, duties, posObserved, A, B, g, C, D, W, V);
	auto end = timeGetTime();
	std::cout << "state(after) : " << state.transpose() << std::endl;
	std::cout << "covError(after) : \n" << covError << std::endl;
	std::cout << "time : " << end - start << std::endl;
	Eigen::Vector3f pos_next = state.head(3);
	Eigen::Vector3f vel_next = state.tail(3);
	objPtr->updateStates(timeGetTime(), pos_next, vel_next);
	objPtr->covError = covError;

	return 0;
	//original codes comes below here
	/*
	odcs odcs;
	std::cout << "ODCS Initializing..." << std::endl;
	odcs.Initialize();
	
	odcs.RegisterObject(FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1350))); // add object
	odcs.StartControl();
	objPtrs[0]->updateStatesTarget(Eigen::Vector3f(0, 0, 1350), Eigen::Vector3f(0, 0, 0));

	std::cout << "Press any key to close." << std::endl;
	getchar();
	odcs.Close();
	return 0;
	*/
	
}
