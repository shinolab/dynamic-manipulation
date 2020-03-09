#include "odcs.hpp"
#include <algorithm>
#include <vector>
#include <deque>
#include <atlbase.h>
#include <Windows.h>
#include <mutex>
#include <shared_mutex>
#include <Eigen\Geometry>

using namespace dynaman;

namespace {
	const float pi = 3.14159265359;
}

FloatingObject::FloatingObject(Eigen::Vector3f const &_positionTarget,
	Eigen::Vector3f const &lowerbound,
	Eigen::Vector3f const &upperbound,
	float _additionalMass,
	float _radius)
	:position(_positionTarget),
	velocity(Eigen::Vector3f::Zero()),
	integral(Eigen::Vector3f::Zero()),
	_lowerbound(lowerbound),
	_upperbound(upperbound),
	additionalMass(_additionalMass),
	radius(_radius),
	isTracked(false),
	lastDeterminationTime(0),
	velocityBufferSize(3),
	positionBuffer(velocityBufferSize, _positionTarget),
	velocityBuffer(velocityBufferSize, Eigen::Vector3f::Zero()),
	dTBuffer(velocityBufferSize, 1),
	covError(100.f * Eigen::MatrixXf::Identity(6, 6)),
	trajectoryPtr(std::make_shared<TrajectoryConstantState>(_positionTarget)){}

FloatingObjectPtr FloatingObject::Create(Eigen::Vector3f const &posTgt, Eigen::Vector3f const &lowerbound, Eigen::Vector3f const &upperbound, float _additionalMass, float _radius)
{
	return FloatingObjectPtr(new FloatingObject(posTgt, lowerbound, upperbound, _additionalMass, _radius));
}

float FloatingObject::sphereMass()
{
	return 1.293f * 4.0f * pi * radius * radius * radius / 3.0f * 1e-9;
}

float FloatingObject::Radius() {
	return radius;
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
	positionBuffer.push_back(positionNew);
	positionBuffer.pop_front();
	velocityBuffer.push_back(velocity);
	velocityBuffer.pop_front();
	if (IsTracked())
	{
		integral += (0.5f * (positionNew + position) - getPositionTarget()) * dt;
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
		integral += (0.5f * (positionNew + position) - getPositionTarget()) * dt;
	}
	velocity = velocityNew;
	position = positionNew;
	lastDeterminationTime = determinationTime;
	dTBuffer.push_back(dt);
	dTBuffer.pop_front();
	positionBuffer.push_back(positionNew);
	positionBuffer.pop_front();
	velocityBuffer.push_back(velocity);
	velocityBuffer.pop_front();
}

void FloatingObject::updateStatesTarget(Eigen::Vector3f &_positionTarget, Eigen::Vector3f &_velocityTarget, Eigen::Vector3f &_accelTarget)
{
	auto constTrajPtr = std::make_shared<TrajectoryConstantState>(_positionTarget, _velocityTarget, _accelTarget);
	SetTrajectory(constTrajPtr);
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

Eigen::Vector3f FloatingObject::AveragePosition() {
	Eigen::Vector3f posAverage(0, 0, 0);
	std::lock_guard<std::mutex> lock(mtxState);
	for (auto itr = positionBuffer.begin(); itr != positionBuffer.end(); itr++) {
		posAverage += *itr;
	}
	return posAverage / 3.f;
}

Eigen::VectorXf FloatingObject::getLatestInput()
{
	return inputLatest;
}

void FloatingObject::setLatestInput(Eigen::VectorXf input)
{
	inputLatest = input;
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


Eigen::Vector3f FloatingObject::lowerbound() {
	return _lowerbound;
}

Eigen::Vector3f FloatingObject::upperbound() {
	return _upperbound;
}