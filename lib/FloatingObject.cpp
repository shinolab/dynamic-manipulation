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
		m_lastDeterminationTime(0),
		velocityBufferSize(3),
		positionBuffer(velocityBufferSize, positionTarget),
		velocityBuffer(velocityBufferSize, Eigen::Vector3f::Zero()),
		dTBuffer(velocityBufferSize, 1),
		m_pTrajectory(std::make_shared<TrajectoryConstantState>(positionTarget)
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

	DWORD FloatingObject::lastDeterminationTime() {
		std::lock_guard<std::mutex> lock(mtxState);
		return m_lastDeterminationTime;
	}

	Eigen::Vector3f FloatingObject::position()
	{
		std::lock_guard<std::mutex> lock(mtxState);
		return m_position;
	}

	Eigen::Vector3f FloatingObject::velocity()
	{
		std::lock_guard<std::mutex> lock(mtxState);
		return averageVelocity(this->velocityBuffer, this->dTBuffer);
	}

	Eigen::Vector3f FloatingObject::integral()
	{
		std::lock_guard<std::mutex> lock(mtxState);
		return m_integral;
	}

	Eigen::Vector3f FloatingObject::positionTarget(DWORD systime_ms)
	{
		std::shared_lock<std::shared_mutex> lock(mtxTrajectory);
		return m_pTrajectory->pos(systime_ms);
	}

	Eigen::Vector3f FloatingObject::velocityTarget(DWORD systime_ms)
	{
		std::shared_lock<std::shared_mutex> lock(mtxTrajectory);
		return m_pTrajectory->vel(systime_ms);
	}

	Eigen::Vector3f FloatingObject::accelTarget(DWORD systime_ms)
	{
		std::shared_lock<std::shared_mutex> lock(mtxTrajectory);
		return m_pTrajectory->accel(systime_ms);
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
		float dt = (float)(determinationTime - m_lastDeterminationTime) / 1000.f; // [sec]
		m_velocity = (positionNew - m_position) / dt;
		dTBuffer.push_back(dt);
		dTBuffer.pop_front();
		positionBuffer.push_back(positionNew);
		positionBuffer.pop_front();
		velocityBuffer.push_back(m_velocity);
		velocityBuffer.pop_front();
		if (isTracked())
		{
			m_integral += (0.5f * (positionNew + m_position) - positionTarget()) * dt;
		}
		m_position = positionNew;
		m_lastDeterminationTime = determinationTime;
	}

	void FloatingObject::resetIntegral() {
		std::lock_guard<std::mutex> lock(mtxState);
		m_integral.setZero();
	}

	void FloatingObject::getStatesTarget(
		Eigen::Vector3f& posTgt,
		Eigen::Vector3f& velTgt,
		Eigen::Vector3f& accelTgt,
		DWORD time
	) {
		std::lock_guard<std::shared_mutex> lock(mtxTrajectory);
		posTgt = m_pTrajectory->pos(time);
		velTgt = m_pTrajectory->vel(time);
		accelTgt = m_pTrajectory->accel(time);
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


	bool FloatingObject::isInsideWorkspace()
	{
		auto pos = position();
		Eigen::Vector3f v0 = pos - lowerbound();
		Eigen::Vector3f v1 = pos - upperbound();
		return (v0.x() * v1.x() <= 0) && (v0.y() * v1.y() <= 0) && (v0.z() * v1.z() <= 0);
	}

	void FloatingObject::setTrajectory(std::shared_ptr<Trajectory> newTrajectoryPtr)
	{
		std::lock_guard<std::shared_mutex> lock(mtxTrajectory);
		m_pTrajectory = newTrajectoryPtr;
	}

	Eigen::Vector3f FloatingObject::lowerbound() {
		return m_lowerbound;
	}

	Eigen::Vector3f FloatingObject::upperbound() {
		return m_upperbound;
	}

	std::string FloatingObject::logCtrlHeader() const {
		return "sys_time,x,y,z,vx,vy,vz,ix,iy,iz,xTgt,yTgt,zTgt,vxTgt,vyTgt,vzTgt,axTgt,ayTgt,azTgt";
	}

	std::ofstream& operator<<(std::ostream& ofs, FloatingObjectPtr pObject) {
		DWORD time = pObject->lastDeterminationTime();
		Eigen::Vector3f pos, vel, integ, posTgt, velTgt, accelTgt;
		pObject->getStates(pos, vel, integ);
		pObject->getStatesTarget(posTgt, velTgt, accelTgt, time);
		ofs
			<< time << ","
			<< pos.x() << "," << pos.y() << "," << pos.z() << ","
			<< vel.x() << "," << vel.y() << "," << vel.z() << ","
			<< integ.x() << "," << integ.y() << "," << integ.z() << ","
			<< posTgt.x() << "," << posTgt.y() << "," << posTgt.z() << ","
			<< velTgt.x() << "," << velTgt.y() << "," << velTgt.z() << ","
			<< accelTgt.x() << "," << accelTgt.y() << "," << accelTgt.z();
	}
}