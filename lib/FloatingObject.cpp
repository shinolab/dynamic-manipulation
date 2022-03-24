#include "FloatingObject.hpp"
#include <algorithm>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <Eigen\Geometry>

#define NOMINMAX
#include <Windows.h>

namespace {
	const float pi = 3.14159265359;
	constexpr float AIR_DENSITY = 1.293e-9; // [kg/mm3]
}

namespace dynaman {
	FloatingObject::FloatingObject(
		const Eigen::Vector3f& positionTarget,
		const Eigen::Vector3f& lowerbound,
		const Eigen::Vector3f& upperbound,
		float radius,
		float weight
	):
		m_position(positionTarget),
		m_velocity(Eigen::Vector3f::Zero()),
		m_integral(Eigen::Vector3f::Zero()),
		m_lowerbound(lowerbound),
		m_upperbound(upperbound),
		m_weight(weight),
		m_radius(radius),
		m_is_tracked(false),
		lastDeterminationTime(0),
		velocityBufferSize(3),
		positionBuffer(velocityBufferSize, positionTarget),
		velocityBuffer(velocityBufferSize, Eigen::Vector3f::Zero()),
		dTBuffer(velocityBufferSize, 1),
		trajectoryPtr(std::make_shared<TrajectoryConstantState>(positionTarget)
		)
	{}

	FloatingObjectPtr FloatingObject::Create(
		const Eigen::Vector3f& posTgt,
		const Eigen::Vector3f& lowerbound,
		const Eigen::Vector3f& upperbound,
		float radius,
		float weight
	){
		return std::make_shared<FloatingObject>(posTgt, lowerbound, upperbound, radius, weight);
	}

	float FloatingObject::sphereMass()
	{
		return AIR_DENSITY * 4.0f * pi * m_radius * m_radius * m_radius / 3.0f;
	}

	float FloatingObject::radius() {
		return m_radius;
	}

	float FloatingObject::weight()
	{
		return m_weight;
	}

	float FloatingObject::totalMass()
	{
		return sphereMass() + weight();
	}

	Eigen::Vector3f FloatingObject::getPosition()
	{
		std::lock_guard<std::mutex> lock(mtxState);
		return m_position;
	}

	Eigen::Vector3f FloatingObject::getVelocity()
	{
		std::lock_guard<std::mutex> lock(mtxState);
		return averageVelocity(this->velocityBuffer, this->dTBuffer);
	}

	Eigen::Vector3f FloatingObject::getIntegral()
	{
		std::lock_guard<std::mutex> lock(mtxState);
		return m_integral;
	}

	Eigen::Vector3f FloatingObject::getPositionTarget(DWORD systime_ms)
	{
		std::shared_lock<std::shared_mutex> lock(mtxTrajectory);
		return trajectoryPtr->pos(systime_ms);
	}

	Eigen::Vector3f FloatingObject::getVelocityTarget(DWORD systime_ms)
	{
		std::shared_lock<std::shared_mutex> lock(mtxTrajectory);
		return trajectoryPtr->vel(systime_ms);
	}

	Eigen::Vector3f FloatingObject::getAccelTarget(DWORD systime_ms)
	{
		std::shared_lock<std::shared_mutex> lock(mtxTrajectory);
		return trajectoryPtr->accel(systime_ms);
	}

	void FloatingObject::getStates(
		Eigen::Vector3f& pos,
		Eigen::Vector3f& vel,
		Eigen::Vector3f& integ
	) {
		std::lock_guard<std::mutex> lock(mtxState);
		pos = m_position;
		vel = averageVelocity(this->velocityBuffer, this->dTBuffer);
		integ = m_integral;
	}

	void FloatingObject::updateStates(DWORD determinationTime, const Eigen::Vector3f& positionNew)
	{
		std::lock_guard<std::mutex> lock(mtxState);
		float dt = (float)(determinationTime - lastDeterminationTime) / 1000.f; // [sec]
		m_velocity = (positionNew - m_position) / dt;
		dTBuffer.push_back(dt);
		dTBuffer.pop_front();
		positionBuffer.push_back(positionNew);
		positionBuffer.pop_front();
		velocityBuffer.push_back(m_velocity);
		velocityBuffer.pop_front();
		if (isTracked())
		{
			m_integral += (0.5f * (positionNew + m_position) - getPositionTarget()) * dt;
		}
		m_position = positionNew;
		lastDeterminationTime = determinationTime;
	}

	void FloatingObject::updateStates(DWORD determinationTime, const Eigen::Vector3f& positionNew, const Eigen::Vector3f& velocityNew)
	{
		std::lock_guard<std::mutex> lock(mtxState);
		float dt = (float)(determinationTime - lastDeterminationTime) / 1000.0;
		if (isTracked())
		{
			m_integral += (0.5f * (positionNew + m_position) - getPositionTarget()) * dt;
		}
		m_position = positionNew;
		m_velocity = velocityNew;
		lastDeterminationTime = determinationTime;
		dTBuffer.push_back(dt);
		dTBuffer.pop_front();
		positionBuffer.push_back(positionNew);
		positionBuffer.pop_front();
		velocityBuffer.push_back(m_velocity);
		velocityBuffer.pop_front();
	}

	void FloatingObject::resetIntegral() {
		std::lock_guard<std::mutex> lock(mtxState);
		m_integral.setZero();
	}

	void FloatingObject::updateStatesTarget(
		const Eigen::Vector3f& _positionTarget,
		const Eigen::Vector3f& _velocityTarget,
		const Eigen::Vector3f& _accelTarget)
	{
		auto constTrajPtr = TrajectoryConstantState::Create(_positionTarget, _velocityTarget, _accelTarget);
		setTrajectory(constTrajPtr);
	}

	bool FloatingObject::isTracked() {
		std::lock_guard<std::mutex> lock(mtxTrack);
		return m_is_tracked;
	}

	void FloatingObject::setTrackingStatus(bool is_tracked) {
		std::lock_guard<std::mutex> lock(mtxTrack);
		this->m_is_tracked = is_tracked;
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

	Eigen::Vector3f FloatingObject::averageVelocity(
		std::deque<Eigen::Vector3f> velocityBuffer,
		std::deque<float> intervalBuffer
	) {
		auto itrVel = velocityBuffer.begin();
		auto itrDT = intervalBuffer.begin();
		Eigen::Vector3f distSum(0.f, 0.f, 0.f);
		float period = 0;
		while (itrVel != velocityBuffer.end())
		{
			distSum += (*itrDT) * (*itrVel);
			period += *itrDT;
			itrDT++;
			itrVel++;
		}
		return distSum /= period;
	}

	Eigen::Vector3f FloatingObject::averagePosition() {
		Eigen::Vector3f posAverage(0, 0, 0);
		std::lock_guard<std::mutex> lock(mtxState);
		for (auto itr = positionBuffer.begin(); itr != positionBuffer.end(); itr++) {
			posAverage += *itr;
		}
		return posAverage / positionBuffer.size();
	}

	bool FloatingObject::isConverged(float tolPos, float tolVel)
	{
		return ((getPosition() - getPositionTarget()).norm() < tolPos) && ((this->getVelocity() - this->getVelocityTarget()).norm() < tolVel);
	}

	bool FloatingObject::isInsideWorkspace()
	{
		auto pos = getPosition();
		Eigen::Vector3f v0 = pos - lowerbound();
		Eigen::Vector3f v1 = pos - upperbound();
		return (v0.x() * v1.x() <= 0) && (v0.y() * v1.y() <= 0) && (v0.z() * v1.z() <= 0);
	}

	void FloatingObject::setTrajectory(std::shared_ptr<Trajectory> newTrajectoryPtr)
	{
		std::lock_guard<std::shared_mutex> lock(mtxTrajectory);
		trajectoryPtr = newTrajectoryPtr;
	}


	Eigen::Vector3f FloatingObject::lowerbound() {
		return m_lowerbound;
	}

	Eigen::Vector3f FloatingObject::upperbound() {
		return m_upperbound;
	}
}