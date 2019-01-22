#include "odcs.hpp"
#include "rootFindingPoly.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>

//input

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
	Eigen::Vector3f force = -Kp * dPos - Kd * dVel - Ki * dInt;
	return force;
}

int main()
{
	std::ofstream ofs("20190121_bangbang_log.csv");
	/*condition*/
	float additionalMass = 1.0e-4;
	float airMass = 5.4e-3;
	float totalMass = additionalMass + airMass;
	
	/*terminal condition*/
	float hf = 1.5f;//[m]
	float vf = 0.3f;//[m/s]

	/*control parameters*/
	float duty_limit = 0.8;
	float kpv = 1.0f;
	float kdv = 2.0f * sqrtf(kpv);
	float kiv = 0.182f * powf(kpv, 1.5f);
	Eigen::Matrix3f gainPhase1; gainPhase1 << kpv * Eigen::Vector3f::Ones()
		, kdv * Eigen::Vector3f::Ones()
		, kiv*Eigen::Vector3f(1.0f, 1.0f, 1.0f);

	Eigen::Matrix3f gainPhase2; gainPhase1 << kpv * Eigen::Vector3f::Ones()
		, kdv * Eigen::Vector3f::Ones()
		, kiv*Eigen::Vector3f(1.0f, 1.0f, 0.0f);

	//==========phase 0: Compute restingPosition==========
	//polynomial to solve
	float A3 = 1.0124e1f;
	float A2 = -8.9171e1f;
	float A1 = 4.0f * (1.0532e-2f - additionalMass * 9.80665f / duty_limit) / (1.8445e-4f);
	float A0 = -hf*hf*hf*hf - A3 * hf*hf*hf - A2 * hf*hf - A1 * hf + 2.0f * totalMass * vf * vf / (1.8455e-4f) / duty_limit;
	
	float h0 = findRootQuarticPoly(A0, A1, A2, A3, 1.0f, hf, 0.0f, 2.0f * hf);

	std::cout << "A0: " << A0 << ", A1: " << A1 << ", A2: " << A2 << ", A3: " << A3 << std::endl;
	std::cout << "terminal state: (" << hf << ", " << vf << "), initial height: " << h0 << std::endl;
	
	//phase I: move to stand-by point
	odcs odcs;
	odcs.Initialize();
	auto objPtr = FloatingObject::Create(Eigen::Vector3f(0, 0, 1000.0f*h0), additionalMass);
	float tolPos = 30.0f;
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
			Eigen::VectorXi amplitudes = (510 / M_PI * duties.array().sqrt().asin().max(0).min(255)).matrix().cast<int>();
			odcs.ocs.CreateFocusOnCenter(objPtr, amplitudes);
			distBuffer.push_back((posObserved-objPtr->getPositionTarget()).norm());
			if (distBuffer.size() > 10) { distBuffer.erase(distBuffer.begin()); }
			ofs << observationTime << ", " << posObserved.x() << ", " << posObserved.y() << ", " << posObserved.z()
				<< ", " << amplitudes[0] << ", " << amplitudes[1] << ", " << amplitudes[2] << ", " << amplitudes[3] << ", " << amplitudes[4] << std::endl;
			std::cout << *std::max_element(distBuffer.begin(), distBuffer.end()) << " size: " << distBuffer.size() << std::endl;
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
	float tolVelTgt = 10.0f;
	//Phase II: follow profile
	std::cout << "Phase II started" << std::endl;
	Eigen::Vector3f posTgt(0, 0, 1000.0f * hf);
	Eigen::Vector3f velTgt(0, 0, 1000.0f * vf);
	Eigen::VectorXf duty_forward(5); duty_forward << duty_limit, .0f, .0f, .0f, .0f;
	Eigen::Matrix3f directions; directions.setIdentity();
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
			force = ComputeAnisotropicPIDForce(objPtr, directions, gainPhase1);
			//force = odcs.ocs.ComputePIDForce(objPtr);
			//Find Control parameters
			Eigen::VectorXf duties = odcs.ocs.FindDutyQP(force, objPtr->getPosition()) + duty_forward;
			Eigen::VectorXi amplitudes = (510 / M_PI * duties.array().sqrt().asin()).cast<int>().max(0).min(255).matrix();
			odcs.ocs.CreateFocusOnCenter(objPtr, amplitudes);
			ofs << observationTime << ", " << posObserved.x() << ", " << posObserved.y() << ", " << posObserved.z()
				<< ", " << amplitudes[0] << ", " << amplitudes[1] << ", " << amplitudes[2] << ", " << amplitudes[3] << ", " << amplitudes[4] << std::endl;
		}
		else if (observationTime - objPtr->lastDeterminationTime > 1000)
		{
			objPtr->isTracked = false;
		}
		if (posObserved.z() > 1.2f * posTgt.z())
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