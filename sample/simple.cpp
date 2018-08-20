#include "odcs.hpp"
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
	ocs ocs; ocs.Initialize();
	ods ods; ods.Initialize();
	//ocs.SetGain(-1.6*Eigen::Vector3f::Ones(), -2.53 * Eigen::Vector3f::Ones(), -0.1*Eigen::Vector3f::Ones());
	std::cout << "positionAUTD :\n" << ocs.positionAUTD << std::endl;
	std::cout << "eulerAnglesAUTD :\n" << ocs.eulerAnglesAUTD << std::endl;
	FloatingObjectPtr objPtr(new FloatingObject(Eigen::Vector3f(100, 0, 1085)));
	objPtr->updateStates(timeGetTime(), Eigen::Vector3f(0, 0, 1085), Eigen::Vector3f(0, 0, 0));
	Eigen::MatrixXf posRel = objPtr->getPosition().replicate(1, 5) - ocs.centerAUTD;
	ocs.SetArfModel(std::unique_ptr<arfModelLinearBase>(new arfModelConstant()));
	Eigen::MatrixXf arf = ocs.arfModelPtr->arf(objPtr->getPosition().replicate(1, 5) - ocs.centerAUTD, ocs.eulerAnglesAUTD);
	std::cout << "ARF : \n" << arf << std::endl;
	ocs.SetArfModel(std::unique_ptr<arfModelLinearBase>(new arfModelTheoreticalTable()));
	arf = ocs.arfModelPtr->arf(objPtr->getPosition().replicate(1, 5) - ocs.centerAUTD, ocs.eulerAnglesAUTD);
	std::cout << "ARF : \n" << arf << std::endl;
	std::cout << "ARF directions :\n" << arf.colwise().normalized() << std::endl;
	std::cout << "ARF norms :\n" << arf.colwise().norm() << std::endl;
	std::cout << "relative directions :\n" << posRel.colwise().normalized() << std::endl;
	Eigen::Vector3f force = ocs.ComputePIDForce(objPtr);
	std::cout << "force to apply : " << force.transpose() << std::endl;
	Eigen::VectorXf amplitudes = ocs.FindDutyQP(force, objPtr->getPosition());
	std::cout << "amplitudes : " << amplitudes.transpose() << std::endl;
	Eigen::Vector3f forceResult = ocs.arfModelPtr->arf(posRel, ocs.eulerAnglesAUTD) * amplitudes;
	std::cout << "resultant force : " << forceResult.transpose() << std::endl;
	return 0;
	//original codes comes below here
	/*
	odcs odcs;
	std::cout << "ODCS Initializing..." << std::endl;
	odcs.Initialize();

	std::vector<FloatingObjectPtr> objPtrs;
	objPtrs.push_back(FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1350))));
	//objs.push_back(FloatingObject(Eigen::Vector3f(-250, 100, 1485))); // add object
	odcs.StartControl(objPtrs);
	objPtrs[0]->updateStatesTarget(Eigen::Vector3f(0, 0, 1350), Eigen::Vector3f(0, 0, 0));

	std::cout << "Press any key to close." << std::endl;
	getchar();
	odcs.Close();
	return 0;
	*/
	
}
