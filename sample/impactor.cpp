#include "odcs.hpp"
#include "rootFindingPoly.hpp"
#include "profile.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include <chrono>

Eigen::Vector3f ComputeAnisotropicPIDForce(FloatingObjectPtr objPtr
	, Eigen::Matrix3f directions
	, Eigen::Matrix3f gains)
{
	Eigen::Matrix3f Kp = directions * gains.col(0).asDiagonal();
	Eigen::Matrix3f Kd = directions * gains.col(1).asDiagonal();
	Eigen::Matrix3f Ki = directions * gains.col(2).asDiagonal();
	Eigen::Vector3f dPos = objPtr->getPosition() - objPtr->getPositionTarget();
	Eigen::Vector3f dVel = objPtr->getVelocity() - objPtr->getVelocityTarget();
	Eigen::Vector3f dInt = objPtr->getIntegral();
	Eigen::Vector3f acceleration = -Kp * dPos - Kd * dVel - Ki * dInt;
	Eigen::Vector3f force = objPtr->totalMass() * acceleration + objPtr->additionalMass * Eigen::Vector3f(0, 0, 9.80665e3f);
	return force;
}

float deriveRestingPosition(float const &hf, float const &vf, float const &additionalMass, float const &totalMass, float const &duty_limit)
{
	float A3 = 1.0124e1f;
	float A2 = -8.9171e1f;
	float A1 = 4.0f * (1.0532e-2f - additionalMass * 9.80665f / duty_limit) / (1.8445e-4f);
	float A0 = -hf*hf*hf*hf - A3 * hf*hf*hf - A2 * hf*hf - A1 * hf + 2.0f * totalMass * vf * vf / (1.8455e-4f) / duty_limit;

	return findRootQuarticPoly(A0, A1, A2, A3, 1.0f, hf, 0.0f, 2.0f * hf);
}

int main()
{
	std::ofstream ofs("20190123_bangbang_log_3.csv");
	/*condition*/
	float additionalMass = 0.095e-4;
	float airMass = 5.4e-3;
	float totalMass = additionalMass + airMass;

	//==========phase 0: Compute restingPosition==========

	Eigen::Vector3f pos0(0.0f, 0.0f, 1300.0f);
	Eigen::Vector3f pos1(300.0f, 0.0f, 1300.0f);

	//phase I: move to stand-by point
	ofs << "time, x, y, z, xTgt, yTgt, zTgt, vxTgt, vyTgt, vzTgt, u0, u1, u2, u3, u4, Fxf, Fyf, Fzf, Fxb, Fyb, Fzb" << std::endl;
	odcs odcs;
	odcs.Initialize();
	odcs.ocs.SetGain(Eigen::Vector3f::Constant(-1.6f), Eigen::Vector3f::Constant(-4.0f), Eigen::Vector3f::Constant(-0.05f));
	auto objPtr = FloatingObject::Create(pos0, additionalMass);
	
	float tolPos = 50.0f;
	float tolVel = 1.0f;
	
	//use wait_until
	std::vector<float> distBuffer;
	while (1)
	{
		Eigen::Vector3f posObserved;
		bool succeeded = odcs.ods.GetPositionByDepth(objPtr, posObserved, true);
		DWORD observationTime = timeGetTime();
		Eigen::Vector3f force; // for log
		if (succeeded && odcs.ods.isInsideWorkSpace(posObserved))
		{
			//----------Determination----------
			objPtr->updateStates(observationTime, posObserved);
			objPtr->isTracked = true;
			//PIDController
			force = odcs.ocs.ComputePIDForce(objPtr);
			//Find Control parameters
			Eigen::VectorXf duties = odcs.ocs.FindDutyQP(force, objPtr->getPosition());
			Eigen::VectorXi amplitudes = (510.0 / M_PI * duties.array().sqrt().asin().max(0).min(255)).matrix().cast<int>();

			odcs.ocs.CreateFocusOnCenter(objPtr, amplitudes);
			distBuffer.push_back((posObserved-pos0).norm());
			if (distBuffer.size() > 30) { distBuffer.erase(distBuffer.begin()); }
			ofs << observationTime << ", " << posObserved.x() << ", " << posObserved.y() << ", " << posObserved.z() << ", " << pos0.x() << ", " << pos0.y() << ", " << pos0.z() << ",0,0,0"
				<< ", " << amplitudes[0] << ", " << amplitudes[1] << ", " << amplitudes[2] << ", " << amplitudes[3] << ", " << amplitudes[4] << ",0,0,0, " << force.x() << ", " << force.y() << ", " << force.z() <<  std::endl;
			std::cout << *std::max_element(distBuffer.begin(), distBuffer.end()) << " size: " << distBuffer.size() << std::endl;
		}
		else if (observationTime - objPtr->lastDeterminationTime > 1000)
		{
			objPtr->isTracked = false;
		}
		if (distBuffer.size() == 30 && *std::max_element(distBuffer.begin(), distBuffer.end()) < tolPos)
		{
			break;
		}
	}
	float tolVelTgt = 10.0f;
	//Phase II: follow profile
	std::cout << "Phase II started" << std::endl;
	profileBangBang profile(10.0f, timeGetTime() / 1000.0f, pos0, pos1);
	//Eigen::Matrix3f directions; directions.setIdentity();
	distBuffer.resize(0);
	while (1)
	{
		Eigen::Vector3f posObserved;
		bool succeeded = odcs.ods.GetPositionByDepth(objPtr, posObserved, true);
		DWORD observationTime = timeGetTime();
		if (succeeded && odcs.ods.isInsideWorkSpace(posObserved))
		{
			//----------Determination----------
			objPtr->updateStates(observationTime, posObserved);
			objPtr->isTracked = true;
			//PIDController
			Eigen::Vector3f force_forward = profile.accelTgt() * objPtr->totalMass();
			Eigen::Vector3f posTgt = profile.posTgt();
			Eigen::Vector3f velTgt = profile.velTgt();
			objPtr->updateStatesTarget(posTgt, velTgt);
			//force = ComputeAnisotropicPIDForce(objPtr, directions, gainBB);
			Eigen::Vector3f force_feedback = odcs.ocs.ComputePIDForce(objPtr);
			//Find Control parameters
			Eigen::VectorXf duties = odcs.ocs.FindDutyQP(force_forward + force_feedback, objPtr->getPosition());
			Eigen::VectorXi amplitudes = (510 / M_PI * duties.array().sqrt().asin()).cast<int>().max(0).min(255).matrix();
			odcs.ocs.CreateFocusOnCenter(objPtr, amplitudes);
			ofs << observationTime << ", " << posObserved.x() << ", " << posObserved.y() << ", " << posObserved.z() << ", "
				<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", " << velTgt.x() << ", " << velTgt.y() << ", " << velTgt.z()
				<< ", " << amplitudes[0] << ", " << amplitudes[1] << ", " << amplitudes[2] << ", " << amplitudes[3] << ", " << amplitudes[4] << ", " 
				<< force_forward.x() << ", " << force_forward.y() << ", " << force_forward.z() << ", " << force_feedback.x() << ", " << force_feedback.y() << ", " << force_feedback.z() << std::endl;
			distBuffer.push_back((posObserved - pos1).norm());
			if (distBuffer.size() > 10) { distBuffer.erase(distBuffer.begin()); }

		}
		else if (observationTime - objPtr->lastDeterminationTime > 1000)
		{
			objPtr->isTracked = false;
		}
		if (distBuffer.size() == 10 && *std::max_element(distBuffer.begin(), distBuffer.end()) < tolPos)
		{
			break;
		}
	}

	odcs.Close();
	return 0;
	//Phase III: keep velocity---anisotropic PID control
	//in the vTgt direction : PD control
	//in the other directions : PID control

}