#include "odcs.hpp"
#include <algorithm>
#include <vector>
#include <deque>
#include <atlbase.h>
#include <Windows.h>
#include <Eigen\Geometry>

#define _USE_MATH_DEFINES
#include <math.h>

FloatingObject::FloatingObject(Eigen::Vector3f _positionTarget)
{
	position << 0, 0, 0;
	velocity << 0, 0, 0;
	integral << 0, 0, 0;
	positionTarget = _positionTarget;
	velocityTarget << 0, 0, 0;
	force_offset << 0, 0, 0;
	lastDeterminationTime = 0;
	isTracked = false;
	isControlled = true;
	velocityBuffer.resize(velocityBufferSize);
	dTBuffer.resize(velocityBufferSize);
	for (auto itr = velocityBuffer.begin(); itr != velocityBuffer.end(); itr++)
	{
		itr->setZero();
	}
	for (auto itr = dTBuffer.begin(); itr != dTBuffer.end(); itr++)
	{
		*itr = 1;
	}
	gravityForce << 0, 0, -0 * 9.8; //0.1 g
}

Eigen::Vector3f FloatingObject::getPosition()
{
	std::lock_guard<std::mutex> lock(mtxState);
	return position;
}

Eigen::Vector3f FloatingObject::getVelocity()
{
	std::lock_guard<std::mutex> lock(mtxState);
	return velocity;
}
Eigen::Vector3f FloatingObject::getIntegral()
{
	std::lock_guard<std::mutex> lock(mtxState);
	return integral;
}

Eigen::Vector3f FloatingObject::getPositionTarget()
{
	std::lock_guard<std::mutex> lock(mtxStateTarget);
	return positionTarget;
}

Eigen::Vector3f FloatingObject::getVelocityTarget()
{
	std::lock_guard<std::mutex> lock(mtxStateTarget);
	return velocityTarget;
}

void FloatingObject::updateStates(DWORD determinationTime, Eigen::Vector3f positionNew)
{
	std::lock_guard<std::mutex> lock(mtxState);
	float dt = (float)(determinationTime - lastDeterminationTime) / 1000.0; // [sec]
	velocity = (positionNew - position) / dt;
	dTBuffer.push_back(dt);
	dTBuffer.pop_front();
	velocityBuffer.push_back(velocity);
	velocityBuffer.pop_front();
	if (isTracked)
	{
		integral += (0.5 * (positionNew + position) - positionTarget) * dt;
	}
	position = positionNew;
	lastDeterminationTime = determinationTime;
}

void FloatingObject::updateStatesTarget(Eigen::Vector3f _positionTarget, Eigen::Vector3f _velocityTarget)
{
	std::lock_guard<std::mutex> lock(mtxStateTarget);
	positionTarget = _positionTarget;
	velocityTarget = _velocityTarget;
}

Eigen::Vector3f FloatingObject::averageVelocity()
{
	Eigen::Vector3f averageVelocity(0, 0, 0);
	std::lock_guard<std::mutex> lock(mtxState);
	auto itrVel = velocityBuffer.begin();
	auto itrDT = dTBuffer.begin();
	float period = 0;
	while (itrVel != velocityBuffer.end())
	{
		averageVelocity += (*itrDT) * (*itrVel);
		period += *itrDT;
		itrDT++;
		itrVel++;
	}
	averageVelocity /= period;
	return averageVelocity;
}

bool FloatingObject::isStable()
{
	return (averageVelocity().norm() < speedLimit);
}
