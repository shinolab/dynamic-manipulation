#include "odcs.hpp"
#include <iostream>
#include <fstream>
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

#pragma comment(lib, "winmm.lib")

int main()
{

	std::ofstream ofs("20180812_sequential_QPEq_Lstep_log.csv");
	ofs << "time[ms], x[mm], y[mm], z[mm], vx[mm/s], vy[mm/s], vz[mm/s], Ix[mms], Iy[mms], Iz[mms], x_tgt[mm], y_tgt[mm], z_tgt[mm], u0, u1, u2, u3, u4" << std::endl;
	const int period = 30000;
	const int loopPeriod = 30;
	odcs odcs;
	odcs.Initialize();

	FloatingObjectPtr objPtr(new FloatingObject(Eigen::Vector3f(0, 0, 1500)));
	int initTime = timeGetTime();
	while (1)
	{
		//set a target position
		int loopInitTime = timeGetTime();
		Eigen::Vector3f positionTarget;
		bool exitFlag = false;
		switch ((loopInitTime - initTime) / period)
		{
		case 0:
			positionTarget << 0, 0, 1500; break;
		case 1:
			positionTarget << 0, 0, 1800; break;
		case 2:
			positionTarget << 0, 0, 1500; break;
		case 3:
			positionTarget << 0, 0, 1200; break;
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
		//control sequence
		odcs.ods.DeterminePositionByDepth(objPtr, true);
		Eigen::VectorXf force = odcs.ocs.ComputePIDForce(objPtr);
		Eigen::VectorXf duties = odcs.ocs.FindDutyQP(force, objPtr->getPosition());
		Eigen::VectorXi amplitudes = (510 / M_PI * duties.array().sqrt().asin().max(0).min(255)).matrix().cast<int>();
		odcs.ocs.DirectSemiPlaneWave(objPtr, amplitudes);

		//output log
		Eigen::Vector3f pos; pos << objPtr->getPosition();
		Eigen::Vector3f vel; vel << objPtr->getVelocity();
		Eigen::Vector3f integral; integral << objPtr->getIntegral();
		
		ofs << loopInitTime << ", " << pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
			<< vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
			<< integral.x() << ", " << integral.y() << ", " << integral.z() << ", "
			<< positionTarget.x() << ", " << positionTarget.y() << ", " << positionTarget.z() << ", "
			<< duties[0] << ", " << duties[1] << ", " << duties[2] << ", " << duties[3] << ", " << duties[4] << std::endl;
		
		int waitTime = loopPeriod - (timeGetTime() - loopInitTime);
		Sleep(std::max(waitTime, 0));
	}
	odcs.Close();
	return 0;
}