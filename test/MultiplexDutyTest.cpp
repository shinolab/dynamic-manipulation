#include "odcs.hpp"
#include <Eigen/Dense>
#include <iostream>

int main() {
	odcs dynaman;
	dynaman.Initialize();
	dynaman.AddDevice(Eigen::Vector3f(992.5f, 270.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(992.5f, 790.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(542.5f, 10.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(542.5f, 530.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(542.5f, 1050.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(92.5f, 270.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(92.5f, 790.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(-357.5f, 10.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(-357.5f, 530.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(-357.5f, 1050.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(-807.5f, 270.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	dynaman.AddDevice(Eigen::Vector3f(-807.5f, 790.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	auto objPtr1 = FloatingObject::Create(Eigen::Vector3f(-423.86f, 596.04f + 100, 1431.f), -1.0e-4f);
	//auto objPtr2 = FloatingObject::Create(Eigen::Vector3f(-423.86f, 596.04f + 100, 1431.f), -1.0e-4f);

	dynaman.RegisterObject(objPtr1);
	//dynaman.RegisterObject(objPtr2);
	auto objPtrs = dynaman.objPtrs;

	Eigen::Vector3f gainP = -Eigen::Vector3f::Constant(1.0f);
	Eigen::Vector3f gainD = -Eigen::Vector3f::Constant(1.0f);
	Eigen::Vector3f gainI = -Eigen::Vector3f::Constant(1.0f);
	//Control
	int num_autd = dynaman.ocsPtr->_autd.geometry()->numDevices();
	int num_object = objPtrs.size();
	Eigen::Matrix3Xf positions(3, num_object); Eigen::Matrix3Xf positionsTarget(3, num_object);
	Eigen::Matrix3Xf velocities(3, num_object); Eigen::Matrix3Xf velocitiesTarget(3, num_object);
	Eigen::Matrix3Xf integrals(3, num_object);
	Eigen::Matrix3Xf accelsTarget(3, num_object);
	Eigen::Matrix3Xf forcesToApply(3, num_object);
	for (auto itrObj = objPtrs.begin(); itrObj != objPtrs.end(); itrObj++) {
		int index = std::distance(objPtrs.begin(), itrObj);
		positions.col(index) = (*itrObj)->getPosition();
		Eigen::Vector3f accel
			= gainP.asDiagonal() * ((*itrObj)->getPosition() - (*itrObj)->getPositionTarget())
			+ gainD.asDiagonal() * ((*itrObj)->averageVelocity() - (*itrObj)->getVelocityTarget())
			+ gainI.asDiagonal() * (*itrObj)->getIntegral()
			+ (*itrObj)->getAccelTarget();
		forcesToApply.col(index) = (*itrObj)->totalMass() * accel + (*itrObj)->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665e3f);
	}
	std::cout << "forces to apply:\n" << forcesToApply << std::endl;
	auto duties = dynaman.ocsPtr->FindDutyQPCGAL(forcesToApply, positions);
	auto dutiesMulti = dynaman.ocsPtr->FindDutyQPMulti(forcesToApply, positions);
	std::cout << "duties:\n" << duties << std::endl;
	dynaman.Close();
	return 0;
}