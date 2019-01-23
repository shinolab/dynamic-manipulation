#include "profile.hpp"
#include <Windows.h>

Eigen::Vector3f profileUniAccel::posTgt(float const &time)
{
	DWORD dt = time - timeInit;
	return  posInit + 0.5 * dt * dt * accel;
}

Eigen::Vector3f profileUniAccel::velTgt(float const &time)
{
	return velInit + (time - timeInit) * accel;
}

Eigen::Vector3f profileUniAccel::accelTgt(float const &time)
{
	return accel;
}

Eigen::Vector3f profileBangBang::posTgt(float const &time)
{
	if (time < timeInit) return posInit;
	if (time > timeInit + timeTotal) return posEnd;
	float dt = time - timeInit;
	return (2.0f * dt < timeTotal) ?  0.5f * dt * dt * accelTgt(time) + posInit :  0.5f * (dt - timeTotal) * (dt - timeTotal) * accelTgt(time) + posEnd;
}

Eigen::Vector3f profileBangBang::velTgt(float const &time)
{
	if (time < timeInit || time >(timeInit + timeTotal)) return Eigen::Vector3f::Zero();
	float dt = time - timeInit;
	return 2.0f * dt < timeTotal ? accelTgt(time) * dt : accelTgt(time) * (dt - timeTotal);
}

Eigen::Vector3f profileBangBang::accelTgt(float const &time)
{
	if (time < timeInit || time >(timeInit + timeTotal)) return Eigen::Vector3f::Zero();
	return 	2.0f * (time - timeInit) < timeTotal ? 4.0f * (posEnd - posInit) / timeTotal / timeTotal :  4.0f * (posInit - posEnd) / timeTotal / timeTotal;
}

