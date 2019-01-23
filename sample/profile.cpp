#include "profile.hpp"
#include <Windows.h>

Eigen::Vector3f profileUniAccel::posTgt(DWORD time)
{
	DWORD dt = time - timeInit;
	return  posInit + 0.5 * dt * dt * accel;
}

Eigen::Vector3f profileUniAccel::velTgt(DWORD time)
{
	return velInit + (time - timeInit) * accel;
}

Eigen::Vector3f profileUniAccel::accelTgt(DWORD time)
{
	return accel;
}