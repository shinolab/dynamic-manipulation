#include <vector>
#include <memory>
#define _USE_MATH_DEFINES
#include <math.h>
#include "Trajectory.hpp"
#include <Eigen/Dense>
#define NOMINMAX
#include <Windows.h>

using namespace dynaman;

TrajectoryConstantState::TrajectoryConstantState(Eigen::Vector3f const &positionTarget,
	Eigen::Vector3f const &velocityTarget,
	Eigen::Vector3f const &accelTarget) :posTgt(positionTarget), velTgt(velocityTarget), accelTgt(accelTarget) {}

Eigen::Vector3f TrajectoryConstantState::pos(DWORD sys_time_ms) {
	return posTgt;
}

Eigen::Vector3f TrajectoryConstantState::vel(DWORD sys_time_ms) {
	return velTgt;
}

Eigen::Vector3f TrajectoryConstantState::accel(DWORD sys_time_ms) {
	return accelTgt;
}

std::shared_ptr<Trajectory> TrajectoryConstantState::Create(Eigen::Vector3f const &positionTarget,
	Eigen::Vector3f const &velocityTarget,
	Eigen::Vector3f const &accelTarget) {
	return std::shared_ptr<Trajectory>(new TrajectoryConstantState(positionTarget, velocityTarget, accelTarget));
}

Eigen::Vector3f TrajectoryBangBang::pos(DWORD sys_time_ms)
{
	if (sys_time_ms < _sys_time_init)
		return _posInit;
	if (sys_time_ms > _sys_time_init + _timeTotal * 1000)
		return _posEnd;
	float dt = (sys_time_ms - _sys_time_init) / 1000.f;
	return (2.0f * dt < _timeTotal) ? 0.5f * dt * dt * accel(sys_time_ms) + _posInit : 0.5f * (dt - _timeTotal) * (dt - _timeTotal) * accel(sys_time_ms) + _posEnd;
}

Eigen::Vector3f TrajectoryBangBang::vel(DWORD sys_time_ms)
{
	if (sys_time_ms < _sys_time_init || sys_time_ms > (_sys_time_init + _timeTotal*1000)) return Eigen::Vector3f::Zero();
	float dt = (sys_time_ms - _sys_time_init) / 1000.f;
	return 2.0f * dt < _timeTotal ? accel(sys_time_ms) * dt : accel(sys_time_ms) * (dt - _timeTotal);
}

Eigen::Vector3f TrajectoryBangBang::accel(DWORD sys_time_ms)
{
	if (sys_time_ms < _sys_time_init || sys_time_ms > (_sys_time_init + _timeTotal * 1000)) return Eigen::Vector3f::Zero();
	float dt = (sys_time_ms - _sys_time_init) / 1000.f;
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
	float period_sec,
	float phaseInit,
	DWORD sys_time_init)
	:_center(center),
	_radius(radius),
	_inclination(inclination),
	_raan(raan),
	_omega(2 * M_PI / period_sec),
	_phaseInit(phaseInit),
	_sys_time_init(sys_time_init) {}

std::shared_ptr<TrajectoryCircle> TrajectoryCircle::Create(
	const Eigen::Vector3f& center,
	float radius,
	float inclination,
	float raan,
	float period_sec,
	float phaseInit,
	DWORD sys_time_init) {
	return std::make_shared<TrajectoryCircle>(center, radius, inclination, raan, period_sec, phaseInit, sys_time_init);
}

float TrajectoryCircle::Radius() {
	return _radius;
}

float TrajectoryCircle::Phase(DWORD sys_time_ms) {
	return _phaseInit + (sys_time_ms - _sys_time_init) / 1000.f * _omega;
}

Eigen::Vector3f TrajectoryCircle::pos(DWORD sys_time_ms) {
	return Eigen::AngleAxisf(_raan, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(_inclination, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(Phase(sys_time_ms), Eigen::Vector3f::UnitZ())
		* (_radius * Eigen::Vector3f::UnitX())
		+ _center;
}

Eigen::Vector3f TrajectoryCircle::vel(DWORD sys_time_ms) {
	return Eigen::AngleAxisf(_raan, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(_inclination, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(Phase(sys_time_ms), Eigen::Vector3f::UnitZ())
		* (_omega * _radius * Eigen::Vector3f::UnitY());
}

Eigen::Vector3f TrajectoryCircle::accel(DWORD sys_time_ms) {
	return Eigen::AngleAxisf(_raan, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(_inclination, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(Phase(sys_time_ms), Eigen::Vector3f::UnitZ())
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

TrajectoryInfShape::TrajectoryInfShape(
	const Eigen::Vector3f& center,
	float height,
	float width,
	float period_sec,
	DWORD sys_time_init)
	:_center(center),
	_omega(4.0f * M_PI / period_sec),
	_height(height),
	_width(width),
	_sys_time_init(sys_time_init) {
}

std::shared_ptr<TrajectoryInfShape> TrajectoryInfShape::Create(
	const Eigen::Vector3f& center,
	float period_sec,
	float height,
	float width,
	DWORD sys_time_init) {
	return std::make_shared<TrajectoryInfShape>(
		center,
		period_sec,
		height,
		width,
		sys_time_init);
}

float TrajectoryInfShape::Phase(DWORD sys_time_ms) {
	return (sys_time_ms - _sys_time_init) / 1000.f * _omega;
}

Eigen::Vector3f TrajectoryInfShape::pos(DWORD sys_time_ms) {
	if (fmodf(Phase(sys_time_ms), 4.f * M_PI) < 2.f * M_PI) {
		return Eigen::Vector3f(
			0.25f * _width * (1 - cosf(Phase(sys_time_ms))),
			0.f,
			0.5f * _height * sinf(Phase(sys_time_ms))
		) + _center;
	}
	else {
		return Eigen::Vector3f(
			0.25f * _width * (-1 + cosf(Phase(sys_time_ms))),
			0.f,
			0.5f * _height * sinf(Phase(sys_time_ms))
		) + _center;
	}
}

Eigen::Vector3f TrajectoryInfShape::vel(DWORD sys_time_ms) {
	if (fmodf(Phase(sys_time_ms), 4.f * M_PI) < 2.f * M_PI) {
		return _omega * Eigen::Vector3f(
			0.25f * _width * (sinf(Phase(sys_time_ms))),
			0.f,
			0.5f * _height * cosf(Phase(sys_time_ms))
		);
	}
	else {
		return _omega * Eigen::Vector3f(
			-0.25f * _width * (sinf(Phase(sys_time_ms))),
			0.f,
			0.5f * _height * cosf(Phase(sys_time_ms))
		);
	}
}

Eigen::Vector3f TrajectoryInfShape::accel(DWORD sys_time_ms) {
	if (fmodf(Phase(sys_time_ms), 4.f * M_PI) < 2.f * M_PI) {
		return _omega * _omega * Eigen::Vector3f(
			0.25f * _width * (cosf(Phase(sys_time_ms))),
			0.f,
			-0.5f * _height * sinf(Phase(sys_time_ms))
		);
	}
	else {
		return _omega * _omega * Eigen::Vector3f(
			-0.25f * _width * (cosf(Phase(sys_time_ms))),
			0.f,
			-0.5f * _height * sinf(Phase(sys_time_ms))
		);
	}
}

TrajectoryHeart::TrajectoryHeart(
	const Eigen::Vector3f& center,
	float height,
	float width,
	float period_sec,
	DWORD sys_time_init)
	:_center(center),
	_height(height),
	_width(width),
	_omega(2.f * M_PI / period_sec),
	_sys_time_init(sys_time_init) {}

std::shared_ptr<TrajectoryHeart> TrajectoryHeart::Create(
	const Eigen::Vector3f& center,
	float height,
	float width,
	float period_sec,
	DWORD sys_time_init)
{
	return std::make_shared<TrajectoryHeart>(center, height, width, period_sec, sys_time_init);
}

float TrajectoryHeart::Phase(DWORD sys_time_ms) {
	return _omega * (sys_time_ms - _sys_time_init) / 1000.f;
}

Eigen::Vector3f TrajectoryHeart::pos(DWORD sys_time_ms) {
	return Eigen::Vector3f(
		0.5f * _width * sinf(Phase(sys_time_ms)) * sinf(Phase(sys_time_ms)) * sinf(Phase(sys_time_ms)),
		0.f,
		0.5f * _height
		* (2.6f * cosf(Phase(sys_time_ms))
			- cosf(2.f * Phase(sys_time_ms))
			- 0.4f * cosf(3.f * Phase(sys_time_ms))
			- 0.2f * cosf(4.f * Phase(sys_time_ms))
			+ 0.508f)
	) + _center;
}

Eigen::Vector3f TrajectoryHeart::vel(DWORD sys_time_ms) {
	return _omega * Eigen::Vector3f(
		1.5f * _width * sinf(Phase(sys_time_ms)) * sinf(Phase(sys_time_ms)) * cosf(Phase(sys_time_ms)),
		0.f,
		0.5f * _height * (
			- 2.6f * sinf(Phase(sys_time_ms))
			+ 2.f * sinf(2.f * Phase(sys_time_ms))
			+ 1.2f * sinf(3.f * Phase(sys_time_ms))
			+ 0.8f * sinf(4.f * Phase(sys_time_ms))
			)
	);
}

Eigen::Vector3f TrajectoryHeart::accel(DWORD sys_time_ms) {
	return _omega * _omega * Eigen::Vector3f(
		1.5f * _width * (
			2.f * sinf(Phase(sys_time_ms)) * cosf(Phase(sys_time_ms)) * cosf(Phase(sys_time_ms))
			- sinf(Phase(sys_time_ms)) * sinf(Phase(sys_time_ms)) * sinf(Phase(sys_time_ms))
			),
		0.f,
		0.5f * _height * (
			-2.6f * cosf(Phase(sys_time_ms))
			+ 4.f * cosf(2.f * Phase(sys_time_ms))
			+ 3.6f * cosf(3.f * Phase(sys_time_ms))
			+ 3.2f * cosf(4.f * Phase(sys_time_ms))
			)
	);
}

