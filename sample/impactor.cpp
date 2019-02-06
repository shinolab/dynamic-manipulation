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
	/*condition*/
	Eigen::Vector3f positionTerminal(0.f, 0.f, 1200.f);
	Eigen::Vector3f velocityTerminal(0.f, 0.f, 300.f);
	float gainP = -1.6f, gainD = -4.0f, gainI = -0.2f;
	float additionalMass = 1e-4f;
	Eigen::VectorXf duty_limit(5); duty_limit << 1.f, 1.f, 1.f, 1.f, 1.f;
	//==========phase 0: Compute restingPosition==========

	
	std::ofstream ofs("20190130_maximum_accel.csv");

	//phase I: move to stand-by point
	ofs << "time, x, y, z, xTgt, yTgt, zTgt, vxTgt, vyTgt, vzTgt, u0, u1, u2, u3, u4, Fxf, Fyf, Fzf, Fxb, Fyb, Fzb" << std::endl;
	odcs odcs;
	odcs.Initialize();
	odcs.ocsPtr->SetGain(Eigen::Vector3f::Constant(gainP), Eigen::Vector3f::Constant(gainD), Eigen::Vector3f::Constant(gainI));
	auto objPtr = FloatingObject::Create(Eigen::Vector3f(0.f, 0.f, 1500.f), additionalMass);
	
	float tolPos = 50.0f;
	float tolVel = 1.0f;
	profileMaxAccel profile(positionTerminal, velocityTerminal, duty_limit, odcs.ocsPtr, objPtr, 0.05f);
	objPtr->updateStatesTarget(profile.posInit(), Eigen::Vector3f(0.f, 0.f, 0.f));
	//use wait_until
	std::vector<float> distBuffer;
	int loopPeriod = 30;
	while (1)
	{
		int loopInitTime = timeGetTime();
		Eigen::Vector3f posObserved;
		bool succeeded = odcs.odsPtr->GetPositionByDepth(objPtr, posObserved, true);
		DWORD observationTime = timeGetTime();
		Eigen::Vector3f force; // for log
		if (succeeded && odcs.odsPtr->isInsideWorkSpace(posObserved))
		{
			//----------Determination----------
			objPtr->updateStates(observationTime, posObserved);
			objPtr->isTracked = true;
			//PIDController
			force = odcs.ocsPtr->ComputePIDForce(objPtr);
			//Find Control parameters
			Eigen::VectorXf duties = odcs.ocsPtr->FindDutyQP(force, objPtr->getPosition());
			Eigen::VectorXi amplitudes = (510.f / M_PI * duties.array().max(0.f).min(1.f).sqrt().asin().matrix()).cast<int>();

			odcs.ocsPtr->CreateFocusOnCenter(objPtr, amplitudes);
			distBuffer.push_back((posObserved-profile.posInit()).norm());
			if (distBuffer.size() > 50) { distBuffer.erase(distBuffer.begin()); }
			ofs << observationTime << ", " << posObserved.x() << ", " << posObserved.y() << ", " << posObserved.z() << ", " << profile.posInit().x() << ", " << profile.posInit().y() << ", " << profile.posInit().z() << ",0,0,0"
				<< ", " << amplitudes[0] << ", " << amplitudes[1] << ", " << amplitudes[2] << ", " << amplitudes[3] << ", " << amplitudes[4] << ",0,0,0, " << force.x() << ", " << force.y() << ", " << force.z() <<  std::endl;
			std::cout << *std::max_element(distBuffer.begin(), distBuffer.end()) << " size: " << distBuffer.size() << std::endl;
		}
		else if (observationTime - objPtr->lastDeterminationTime > 1000)
		{
			objPtr->isTracked = false;
		}
		if (distBuffer.size() == 50 && *std::max_element(distBuffer.begin(), distBuffer.end()) < tolPos)
		{
			break;
		}
		int waitTime = loopPeriod - (timeGetTime() - loopInitTime);
		Sleep(std::max(waitTime, 0));
	}
	float tolVelTgt = 10.0f;
	//Phase II: follow profile
	std::cout << "Phase II started" << std::endl;
	odcs.ocsPtr->SetGain(Eigen::Vector3f(gainP, gainP, 0.f), Eigen::Vector3f(gainD, gainD, 0.f), Eigen::Vector3f(gainI, gainI, 0.f));
	//profileBangBang profile(timeTrans, timeGetTime() / 1000.0f, pos0, pos1);
	//profileMaxVerticalVelocity profile(duty_limit);
	//Eigen::Matrix3f directions; directions.setIdentity();
	distBuffer.resize(0);
	while (1)
	{
		int loopInitTime = timeGetTime();
		Eigen::Vector3f posObserved(0, 0, 0);
		bool succeeded = odcs.odsPtr->GetPositionByDepth(objPtr, posObserved, true);
		DWORD observationTime = timeGetTime();
		if (succeeded && odcs.odsPtr->isInsideWorkSpace(posObserved))
		{
			//----------Determination----------
			objPtr->updateStates(observationTime, posObserved);
			objPtr->isTracked = true;
			//PIDController
	
			Eigen::Vector3f posTgt = objPtr->getPosition();// = profile.posTgt(posObserved.z());
			Eigen::Vector3f velTgt = objPtr->getVelocity();// = profile.velTgt(posObserved.z());
			objPtr->updateStatesTarget(posTgt, velTgt);// to apply I control in z direction
			//force = ComputeAnisotropicPIDForce(objPtr, directions, gainBB);
			Eigen::Vector3f direction = (positionTerminal - posObserved).normalized();
			Eigen::Vector3f constDirection1 = direction.cross(Eigen::Vector3f::UnitX());
			if (constDirection1.norm() < 1.0e-6) // in case that velocity direction is parallel to x-axis
			{
				constDirection1 = direction.cross(Eigen::Vector3f::UnitY());
			}
			constDirection1.normalize();
			Eigen::Vector3f constDirection2 = constDirection1.cross(direction);
			Eigen::MatrixXf constDirections(3, 2);
			constDirections << constDirection1, constDirection2;
			float force_forward_norm;
			Eigen::VectorXf duty_forward = odcs.ocsPtr->FindDutyMaximizeForce(Eigen::Vector3f::UnitZ()
				, constDirections
				, posObserved
				, duty_limit
				, force_forward_norm);
			Eigen::MatrixXf posRel = posObserved.replicate(1, odcs.ocsPtr->centersAUTD.cols()) - odcs.ocsPtr->centersAUTD;
			Eigen::Vector3f force_forward = odcs.ocsPtr->arfModelPtr->arf(posRel, odcs.ocsPtr->eulerAnglesAUTD) * duty_forward;
			Eigen::Vector3f force_feedback = odcs.ocsPtr->ComputePIDForce(objPtr);
			//Eigen::VectorXf duty_feedback = odcs.ocsPtr->FindDutyQP(force_feedback, posObserved);
			//Eigen::VectorXf duty_limit = 0.6*(Eigen::VectorXf::Ones(duty_feedback.size()) - duty_feedback);
				
			//Find Control parameters
			Eigen::VectorXf duties = odcs.ocsPtr->FindDutyQP(force_feedback + force_forward, posObserved);
			Eigen::VectorXi amplitudes = (510.f / M_PI * duties.array().max(0.f).min(1.f).sqrt().asin().matrix()).cast<int>();
			odcs.ocsPtr->CreateFocusOnCenter(objPtr, amplitudes);
			Eigen::VectorXf force_forward = odcs.ocsPtr->arfModelPtr->arf(posObserved.replicate(1, odcs.ocsPtr->centersAUTD.cols()) - odcs.ocsPtr->centersAUTD, odcs.ocsPtr->eulerAnglesAUTD) * duty_forward;
			ofs << observationTime << ", " << posObserved.x() << ", " << posObserved.y() << ", " << posObserved.z() << ", "
				<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", " << velTgt.x() << ", " << velTgt.y() << ", " << velTgt.z()
				<< ", " << amplitudes[0] << ", " << amplitudes[1] << ", " << amplitudes[2] << ", " << amplitudes[3] << ", " << amplitudes[4] << ", " 
				<< force_forward.x() << ", " << force_forward.y() << ", " << force_forward.z() << ", " << force_feedback.x() << ", " << force_feedback.y() << ", " << force_feedback.z() << std::endl;
			distBuffer.push_back((posObserved - positionTerminal).norm());
			if (distBuffer.size() > 10) { distBuffer.erase(distBuffer.begin()); }
		}
		else if (observationTime - objPtr->lastDeterminationTime > 1000)
		{
			objPtr->isTracked = false;
		}
		if (posObserved.z() > 1.1 * positionTerminal.z())
		{
			break;
		}		
	}

	odcs.Close();
	return 0;


}