#include "odcs.hpp"
#include <algorithm>
#include <vector>
#include <deque>
#include <atlbase.h>
#include <Windows.h>
#include <mutex>
#include <shared_mutex>
#include <Eigen\Geometry>

#define _USE_MATH_DEFINES
#include <math.h>

FloatingObject::FloatingObject(Eigen::Vector3f const &_positionTarget, float _additionalMass)
{
	position << _positionTarget;
	velocity << 0, 0, 0;
	integral << 0, 0, 0;
	SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryConstantState(_positionTarget)));
	additionalMass = _additionalMass;
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
	covError = 100 * Eigen::VectorXf::Ones(6).asDiagonal();
}

FloatingObjectPtr FloatingObject::Create(Eigen::Vector3f const &posTgt, float _additionalMass)
{
	return FloatingObjectPtr(new FloatingObject(posTgt, _additionalMass));
}

float FloatingObject::sphereMass()
{
	return 1.293 * 4.0 * M_PI * pow(radius/1000, 3) / 3.0;
}

float FloatingObject::AdditionalMass()
{
	return additionalMass;
}

float FloatingObject::totalMass()
{
	return sphereMass() + additionalMass;
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
	std::shared_lock<std::shared_mutex> lock(mtxTrajectory);
	return trajectoryPtr->pos();
}

Eigen::Vector3f FloatingObject::getVelocityTarget()
{
	std::shared_lock<std::shared_mutex> lock(mtxTrajectory);
	return trajectoryPtr->vel();
}

Eigen::Vector3f FloatingObject::getAccelTarget()
{
	std::shared_lock<std::shared_mutex> lock(mtxTrajectory);
	return trajectoryPtr->accel();
}

void FloatingObject::updateStates(DWORD determinationTime, Eigen::Vector3f &positionNew)
{
	std::lock_guard<std::mutex> lock(mtxState);
	float dt = (float)(determinationTime - lastDeterminationTime) / 1000.0; // [sec]
	velocity = (positionNew - position) / dt;
	dTBuffer.push_back(dt);
	dTBuffer.pop_front();
	velocityBuffer.push_back(velocity);
	velocityBuffer.pop_front();
	if (IsTracked())
	{
		integral += (0.5 * (positionNew + position) - getPositionTarget()) * dt;
	}
	position = positionNew;
	lastDeterminationTime = determinationTime;
}

void FloatingObject::updateStates(DWORD determinationTime, Eigen::Vector3f &positionNew, Eigen::Vector3f &velocityNew)
{
	std::lock_guard<std::mutex> lock(mtxState);
	float dt = (float)(determinationTime - lastDeterminationTime) / 1000.0;
	if (IsTracked())
	{
		integral += (0.5 * (positionNew + position) - getPositionTarget()) * dt;
	}
	velocity = velocityNew;
	position = positionNew;
	lastDeterminationTime = determinationTime;
	dTBuffer.push_back(dt);
	dTBuffer.pop_front();
	velocityBuffer.push_back(velocity);
	velocityBuffer.pop_front();
}

void FloatingObject::updateStatesTarget(Eigen::Vector3f &_positionTarget, Eigen::Vector3f &_velocityTarget, Eigen::Vector3f &_accelTarget)
{
	SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryConstantState(_positionTarget, _velocityTarget, _accelTarget)));
}

bool FloatingObject::IsTracked() {
	std::lock_guard<std::mutex> lock(mtxTrack);
	return isTracked;
}

void FloatingObject::SetTrackingStatus(bool _isTracked){
	std::lock_guard<std::mutex> lock(mtxTrack);
	this->isTracked = _isTracked;
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

Eigen::VectorXf FloatingObject::getLatestInput()
{
	return inputLatest;
}

void FloatingObject::setLatestInput(Eigen::VectorXf input)
{
	inputLatest = input;
}

bool FloatingObject::isStable()
{
	return (averageVelocity().norm() < speedLimit);
}

bool FloatingObject::isConverged(float tolPos, float tolVel)
{
	return ((getPosition() - getPositionTarget()).norm() < tolPos) && ((this->getVelocity() - this->getVelocityTarget()).norm() < tolVel);
}

void FloatingObject::SetTrajectory(std::shared_ptr<Trajectory> newTrajectoryPtr)
{
	std::lock_guard<std::shared_mutex> lock(mtxTrajectory);
	trajectoryPtr = newTrajectoryPtr;
}
