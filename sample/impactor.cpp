#include "odcs.hpp"
#include "rootFindingPoly.hpp"
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
	Eigen::Vector3f positionTerminal(0.f, 0.f, 1400.f);
	Eigen::Vector3f velocityTerminal(0.f, 0.f, 300.f);
	float gainP = -1.6f, gainD = -4.0f, gainI = -0.05f;
	float additionalMass = 1e-4f;
	Eigen::VectorXf duty_limit(5); duty_limit.setConstant(0.6f);
	//==========phase 0: Compute restingPosition==========
	
	std::ofstream ofs("20190206_maximum_accel_multitrial.csv");

	//phase I: move to stand-by point
	ofs << "trial, time, x, y, z, xTgt, yTgt, zTgt, vxTgt, vyTgt, vzTgt, u0, u1, u2, u3, u4, Fxf, Fyf, Fzf, Fxb, Fyb, Fzb" << std::endl;
	odcs odcs;
	odcs.Initialize();
	odcs.ocsPtr->SetGain(Eigen::Vector3f::Constant(gainP), Eigen::Vector3f::Constant(gainD), Eigen::Vector3f::Constant(gainI));
	auto objPtr = FloatingObject::Create(Eigen::Vector3f(0.f, 0.f, 1500.f), additionalMass);
	
	float tolPos = 50.0f;
	float tolVel = 1.0f;
	TrajectoryMaxAccel trajectory(positionTerminal, velocityTerminal, 0.f, duty_limit, odcs.ocsPtr, objPtr, 0.05f);

	//use wait_until
	std::vector<float> distBuffer;
	int loopPeriod = 33;
	for (int iTrial = 0; iTrial < 10; iTrial++)
	{
		distBuffer.resize(0);
		std::cout << "TRIAL " << iTrial << std::endl;
		objPtr->updateStatesTarget(trajectory.posInit(), Eigen::Vector3f(0.f, 0.f, 0.f));
		distBuffer.resize(0);
		do {
			if (objPtr->IsTracked())
			{
				distBuffer.push_back((objPtr->getPosition() - trajectory.posInit()).norm());
				if (distBuffer.size() > 50) { distBuffer.erase(distBuffer.begin()); }
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(33));
		} while (distBuffer.size() < 50 || *std::max_element(distBuffer.begin(), distBuffer.end()) < tolPos);
		//Phase II: follow profile
		std::cout << "Phase II started" << std::endl;

		objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryMaxAccel(positionTerminal, velocityTerminal, timeGetTime()/1000.f + 2.f, duty_limit, odcs.ocsPtr,objPtr)));
		do {
			if (objPtr->IsTracked())
			{
				float currentTime = timeGetTime() / 1000.f + *trajectory.pathTime.rbegin();
				Eigen::Vector3f pos = objPtr->getPosition();
				Eigen::Vector3f vel = objPtr->getVelocity();
				Eigen::Vector3f posTgt = objPtr->getPositionTarget();
				Eigen::Vector3f velTgt = objPtr->getVelocityTarget();
				ofs << iTrial << ", " << currentTime << ", " << pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
					<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", "
					<< velTgt.x() << ", " << velTgt.y() << ", " << velTgt.z() << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(33));
		} while (objPtr->getPosition().z() > positionTerminal.z());
	}
	
	odcs.Close();
	return 0;


}