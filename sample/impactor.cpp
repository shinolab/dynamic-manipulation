#include "odcs.hpp"
#include "rootFindingPoly.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

//input
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

int main()
{
	/*condition*/
	float additionalMass = 1.0e-3;
	float airMass = 5.4e-3;
	float totalMass = additionalMass + airMass;
	
	/*terminal condition*/
	float hf = 1.0;//[m]
	float vf = 0.1;//[m/s]
	auto objPtr = FloatingObject::Create(Eigen::Vector3f(0, 0, 600), additionalMass);

	float kpv = 1.0f;
	float kdv = 2.0f * sqrtf(kpv);
	float kiv = 0.182f * powf(kpv, 1.5);
	Eigen::Matrix3f gainPhase1; gainPhase1 << kpv * Eigen::Vector3f::Ones()
		, kdv * Eigen::Vector3f::Ones()
		, kiv*Eigen::Vector3f(1, 1, 1);

	Eigen::Matrix3f gainPhase2; gainPhase1 << kpv * Eigen::Vector3f::Ones()
		, kdv * Eigen::Vector3f::Ones()
		, kiv*Eigen::Vector3f(1, 1, 0);

	//phase 0: Compute restingPosition
	Eigen::Vector3f posRes(0, 0, 300);
	//polynomial to solve
	float A3 = 1.0124e1;
	float A2 = -8.9171e1;
	float A1 = 4.0 * (1.0532e-1 - additionalMass * 9.80665) / 1.8445e-3;
	float A0 = -hf*hf*hf*hf - A3 * hf*hf*hf - A2 * hf*hf - A1 * hf + 2.0f*totalMass*vf*vf / 1.8455e-3;
	
	float h0 = findRootQuarticPoly(A0, A1, A2, A3, 1.0f, hf, 0.0f, 2.0f * hf);
	odcs odcs;
	odcs.Initialize();
	odcs.StartControl();
	
	//phase I: move to stand-by point
	objPtr->updateStatesTarget(posRes, Eigen::Vector3f(0.0f, 0.0f, h0));
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
		Eigen::Vector3f force = ComputeAnisotropicPIDForce(objPtr, Eigen::Matrix3f::Identity(), gainPhase2);
		Eigen::VectorXf duties = odcs.ocs.FindDutyQP(force, objPtr->getPosition());
		Eigen::VectorXi amplitudes = (510 / M_PI * duties.array().pow(1.0f/1.5f).sqrt().asin().max(0).min(255)).matrix().cast<int>();
		odcs.ocs.CreateFocusOnCenter(objPtr, amplitudes);
	}
}