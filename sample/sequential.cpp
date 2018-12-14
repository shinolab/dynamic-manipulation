#include "odcs.hpp"
#include "kalmanFilter.hpp"
#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

#pragma comment(lib, "winmm.lib")

int main()
{
	std::ofstream ofs("20181211_controlByFocus.csv");
	ofs << "time[ms], succeeded, x(raw)[mm], y(raw)[mm], z(raw)[mm], "
		<< "x(est)[mm], y(est)[mm], z(est)[mm], vx[mm/s], vy[mm/s], vz[mm/s], "
		<< "Ix[mms], Iy[mms], Iz[mms], x_tgt[mm], y_tgt[mm], z_tgt[mm], "
		<< "Fx[mN], Fy[mN], Fz[mN], u0, u1, u2, u3, u4" << std::endl;
	const int period = 10000;
	const int loopPeriod = 30;
	odcs odcs;
	odcs.Initialize();
	auto objPtr = FloatingObject::Create(Eigen::Vector3f(0, 0, 1350));
	odcs.RegisterObject(objPtr);
	int initTime = timeGetTime();
	while (1)
	{
		bool exitFlag = false;
		int loopInitTime = timeGetTime();
		//----------set a target position----------
		Eigen::Vector3f positionTarget;
		switch ((loopInitTime - initTime) / period)
		{
		case 0:
			positionTarget << 0, 0, 1500; break;
		case 1:
			positionTarget << 0, 0, 1800; break;
		case 2:
			positionTarget << 0, 0, 1500; break;
		case 3:
			positionTarget << 200, 0, 1500; break;
		case 4:
			positionTarget << 0, 0, 1500; break;
		default:
			exitFlag = true;
			break;
		}
		if (exitFlag)
		{
			break;
		}
		objPtr->updateStatesTarget(positionTarget, Eigen::Vector3f(0, 0, 0));
		
		//----------Observation----------
		Eigen::Vector3f posObserved;
		bool succeeded = odcs.ods.GetPositionByDepth(objPtr, posObserved, true);
		DWORD observationTime = timeGetTime();
		Eigen::Vector3f force; // for log
		if (succeeded && odcs.ods.isInsideWorkSpace(posObserved))
		{
			//----------Determination----------
			//odcs.DetermineStateKF(objPtr, posObserved, observationTime);
			objPtr->updateStates(observationTime, posObserved);
			objPtr->isTracked = true;
			//PIDController
			force = odcs.ocs.ComputePIDForce(objPtr);
			//Find Control parameters
			Eigen::VectorXf duties = odcs.ocs.FindDutyQP(force, objPtr->getPosition());
			Eigen::VectorXi amplitudes = (510 / M_PI * duties.array().sqrt().asin().max(0).min(255)).matrix().cast<int>();
			//odcs.ocs.DirectSemiPlaneWave(objPtr, amplitudes);
			odcs.ocs.CreateFocusOnCenter(objPtr, amplitudes);
			objPtr->setLatestInput(duties);
		}
		else if (observationTime - objPtr->lastDeterminationTime > 1000)
		{
			objPtr->isTracked = false;
		}

		//output log
		Eigen::Vector3f pos; pos << objPtr->getPosition();
		Eigen::Vector3f vel; vel << objPtr->getVelocity();
		Eigen::Vector3f integral; integral << objPtr->getIntegral();
		Eigen::VectorXf u = objPtr->getLatestInput();
		
		ofs << loopInitTime << ", " << succeeded << "," << objPtr->isTracked << ", " << posObserved.x() << ", " << posObserved.y() << ", " << posObserved.z() << ", "
			<< pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
			<< vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
			<< integral.x() << ", " << integral.y() << ", " << integral.z() << ", "
			<< positionTarget.x() << ", " << positionTarget.y() << ", " << positionTarget.z() << ", "
			<< force.x() << ", " << force.y() << ", " << force.z() << ", "
			<< u[0] << ", " << u[1] << ", " << u[2] << ", " << u[3] << ", " << u[4] << std::endl;
		
		int waitTime = loopPeriod - (timeGetTime() - loopInitTime);
		Sleep(std::max(waitTime, 0));
	}
	odcs.Close();
	return 0;
	
}