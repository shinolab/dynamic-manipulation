#include "odcs.hpp"
#include "rootFindingPoly.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

//input
/*
Eigen::Vector3f ComputeAnisotropicPIDForce(FloatingObjectPtr objPtr
	, Eigen::Matrix3f directions
	, Eigen::Matrix3f gains)
{
	
	Eigen::Matrix3f Kp = directions * (gains.col(0)).diagonal();
	Eigen::Matrix3f Kd = directions * (gains.col(1)).diagonal();
	Eigen::Matrix3f Ki = directions * (gains.col(2)).diagonal();
	Eigen::Vector3f dPos = objPtr->getPosition() - objPtr->getPositionTarget();
	Eigen::Vector3f dVel = objPtr->getVelocity() - objPtr->getVelocityTarget();
	Eigen::Vector3f dInt = objPtr->getIntegral();
	Eigen::Vector3f force = -Kp * dPos - Kd * dVel - Ki * dInt;
	return force;
}
*/
int main()
{
	/*condition*/
	float additionalMass = 1.0e-4;
	float airMass = 5.4e-3;
	float totalMass = additionalMass + airMass;
	
	/*terminal condition*/
	float hf = 1.0;//[m]
	float vf = 0.4;//[m/s]
	auto objPtr = FloatingObject::Create(Eigen::Vector3f(0, 0, 1000), additionalMass);

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
	float A1 = 4.0 * (1.0532e-2f - additionalMass * 9.80665f / duty_limit) / (1.8445e-4f);
	float A0 = -hf*hf*hf*hf - A3 * hf*hf*hf - A2 * hf*hf - A1 * hf + 2.0f * totalMass * vf * vf / (1.8455e-4f) / duty_limit;
	
	float h0 = findRootQuarticPoly(A0, A1, A2, A3, 1.0f, hf, 0.0f, 2.0f * hf);

	std::cout << "A0: " << A0 << ", A1: " << A1 << ", A2: " << A2 << ", A3: " << A3 << std::endl;
	std::cout << "terminal state: (" << hf << ", " << vf << "), initial height: " << h0 << std::endl;
	return 0;

	
	//phase I: move to stand-by point
	odcs odcs;
	odcs.Initialize();
	odcs.StartControl();
	objPtr->updateStatesTarget(Eigen::Vector3f(0.0f, 0.0f, h0), Eigen::Vector3f(0, 0, 0));
	float tolPos = 1.0f;
	float tolVel = 1.0f;
	//use wait_until
	
	while (1)
	{
		if (objPtr->isConverged(tolPos, tolVel))
		{
			break;
		}
	}
	float tolVelTgt = 1.0f;
	//Phase II: follow profile
	Eigen::Vector3f posTgt(0, 0, 1000*hf);
	Eigen::Vector3f velTgt(0, 0, 1000*vf);
	while (1)
	{
		if ((objPtr->getVelocity() - velTgt).dot(velTgt.normalized()) > tolVelTgt)
		{
			break;
		}
		Eigen::VectorXf duties_r;
	}
	//Phase III: keep velocity---anisotropic PID control
	//in the vTgt direction : PD control
	//in the other directions : PID control
	Eigen::Vector3f posSwitch = objPtr->getPosition();

	while (1)
	{
		//Eigen::Vector3f force = ComputeAnisotropicPIDForce(objPtr, Eigen::Matrix3f::Identity(), gainPhase2);
		//Eigen::VectorXf duties = odcs.ocs.FindDutyQP(force, objPtr->getPosition());
		//Eigen::VectorXi amplitudes = (510 / M_PI * duties.array().pow(1.0f/1.5f).sqrt().asin().max(0).min(255)).matrix().cast<int>();
		//odcs.ocs.CreateFocusOnCenter(objPtr, amplitudes);
	}
}