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

profileMaxVerticalVelocity::profileMaxVerticalVelocity(float const &duty_limit)
{
	this->duty_limit = duty_limit;
}

Eigen::Vector3f profileMaxVerticalVelocity::posTgt(float const &z)
{
	return Eigen::Vector3f(0.0f, 0.0f, z);
}

Eigen::Vector3f profileMaxVerticalVelocity::velTgt(float const &z)
{
	return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
}

Eigen::Vector3f profileMaxVerticalVelocity::accelTgt(float const &height)
{
	float z = height / 1000.0f;
	float forceMax = ((1.8455e-1f * z + 1.4013f) * z - 8.228f) * z +10.532;
	return duty_limit * forceMax * Eigen::Vector3f::UnitZ();
}

