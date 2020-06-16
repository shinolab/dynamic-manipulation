#include "odcs.hpp"
#include <Eigen/Dense>
#include <Windows.h>
#include <vector>
#include <memory>

using namespace dynaman;

TrajectoryConstantState::TrajectoryConstantState(Eigen::Vector3f const &positionTarget,
	Eigen::Vector3f const &velocityTarget,
	Eigen::Vector3f const &accelTarget) :posTgt(positionTarget), velTgt(velocityTarget), accelTgt(accelTarget) {}

Eigen::Vector3f TrajectoryConstantState::pos(DWORD time) {
	return posTgt;
}

Eigen::Vector3f TrajectoryConstantState::vel(DWORD time) {
	return velTgt;
}

Eigen::Vector3f TrajectoryConstantState::accel(DWORD time) {
	return accelTgt;
}

std::shared_ptr<Trajectory> TrajectoryConstantState::Create(Eigen::Vector3f const &positionTarget,
	Eigen::Vector3f const &velocityTarget,
	Eigen::Vector3f const &accelTarget) {
	return std::shared_ptr<Trajectory>(new TrajectoryConstantState(positionTarget, velocityTarget, accelTarget));
}

Eigen::Vector3f TrajectoryBangBang::pos(DWORD sys_time)
{
	if (sys_time < _sys_time_init)
		return _posInit;
	if (sys_time > _sys_time_init + _timeTotal * 1000)
		return _posEnd;
	float dt = (sys_time - _sys_time_init) / 1000.f;
	return (2.0f * dt < _timeTotal) ? 0.5f * dt * dt * accel(sys_time) + _posInit : 0.5f * (dt - _timeTotal) * (dt - _timeTotal) * accel(sys_time) + _posEnd;
}

Eigen::Vector3f TrajectoryBangBang::vel(DWORD sys_time)
{
	if (sys_time < _sys_time_init || sys_time > (_sys_time_init + _timeTotal*1000)) return Eigen::Vector3f::Zero();
	float dt = (sys_time - _sys_time_init) / 1000.f;
	return 2.0f * dt < _timeTotal ? accel(sys_time) * dt : accel(sys_time) * (dt - _timeTotal);
}

Eigen::Vector3f TrajectoryBangBang::accel(DWORD sys_time)
{
	if (sys_time < _sys_time_init || sys_time > (_sys_time_init + _timeTotal * 1000)) return Eigen::Vector3f::Zero();
	float dt = (sys_time - _sys_time_init) / 1000.f;
	return 	2.0f * dt < _timeTotal ? 4.0f * (_posEnd - _posInit) / _timeTotal / _timeTotal : 4.0f * (_posInit - _posEnd) / _timeTotal / _timeTotal;
}

std::shared_ptr<Trajectory> TrajectoryBangBang::Create(float timeTotal,
	DWORD sys_time_init,
	Eigen::Vector3f const &posInit,
	Eigen::Vector3f const &posEnd) {
	return std::shared_ptr<Trajectory>(new TrajectoryBangBang(timeTotal, sys_time_init, posInit, posEnd));
}

TrajectoryCircle::TrajectoryCircle(
	const Eigen::Vector3f& center,
	float radius,
	float inclination,
	float raan,
	float period,
	float phaseInit,
	DWORD sys_time_init)
	:_center(center),
	_radius(radius),
	_inclination(inclination),
	_raan(raan),
	_omega(2 * M_PI / period),
	_phaseInit(phaseInit),
	_sys_time_init(sys_time_init) {}

std::shared_ptr<TrajectoryCircle> TrajectoryCircle::Create(
	const Eigen::Vector3f& center,
	float radius,
	float inclination,
	float raan,
	float period,
	float phaseInit,
	DWORD sys_time_init) {
	return std::make_shared<TrajectoryCircle>(center, radius, inclination, raan, period, phaseInit, sys_time_init);
}

float TrajectoryCircle::Radius() {
	return _radius;
}

float TrajectoryCircle::Phase(DWORD sys_time) {
	return _phaseInit + (sys_time - _sys_time_init) / 1000.f * _omega;
}

Eigen::Vector3f TrajectoryCircle::pos(DWORD sys_time) {
	return Eigen::AngleAxisf(_raan, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(_inclination, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(Phase(sys_time), Eigen::Vector3f::UnitZ())
		* (_radius * Eigen::Vector3f::UnitX())
		+ _center;
}

Eigen::Vector3f TrajectoryCircle::vel(DWORD sys_time) {
	return Eigen::AngleAxisf(_raan, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(_inclination, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(Phase(sys_time), Eigen::Vector3f::UnitZ())
		* (_omega * _radius * Eigen::Vector3f::UnitY());
}

Eigen::Vector3f TrajectoryCircle::accel(DWORD sys_time) {
	return Eigen::AngleAxisf(_raan, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(_inclination, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(Phase(sys_time), Eigen::Vector3f::UnitZ())
		* (- _omega * _omega * _radius * Eigen::Vector3f::UnitX());
}

TrajectorySinusoid::TrajectorySinusoid(
	const Eigen::Vector3f& direction,
	float amplitude,
	float period,
	const Eigen::Vector3f& center,
	DWORD sys_time_init)
	:_direction(direction),
	_amplitude(amplitude),
	_period(period),
	_center(center),
	_sys_time_init(sys_time_init) {}

std::shared_ptr<TrajectorySinusoid> TrajectorySinusoid::Create(
	const Eigen::Vector3f& direction,
	float amplitude,
	float period,
	const Eigen::Vector3f& center,
	DWORD sys_time_init
) {
	return std::make_shared<TrajectorySinusoid>(direction, amplitude, period, center, sys_time_init);
}

Eigen::Vector3f TrajectorySinusoid::pos(DWORD sys_time) {
	return _center + _direction * _amplitude * sinf((Phase(sys_time)));
}

Eigen::Vector3f TrajectorySinusoid::vel(DWORD sys_time) {
	return (sys_time < _sys_time_init) ? Eigen::Vector3f(0, 0, 0) : _direction * _amplitude * Omega(sys_time) * cosf(Phase(sys_time));
}

Eigen::Vector3f TrajectorySinusoid::accel(DWORD sys_time) {
	return -_direction * _amplitude * Omega(sys_time) * Omega(sys_time) * sinf(Phase(sys_time));
}

float TrajectorySinusoid::Phase(DWORD sys_time) {
	return (sys_time < _sys_time_init) ? 0.f : fmodf((sys_time - _sys_time_init)/1000.f, _period) / _period * M_PI * 2.0f;
}

float TrajectorySinusoid::Omega(DWORD sys_time) {
	return 2.0f * M_PI / _period;
}