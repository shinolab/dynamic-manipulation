#include <vector>
#include <iostream>
#include <memory>
#define _USE_MATH_DEFINES
#include <math.h>
#include "Trajectory.hpp"
#include <Eigen/Dense>
#define NOMINMAX
#include <Windows.h>

using namespace dynaman;

TrajectoryConstantState::TrajectoryConstantState(
	Eigen::Vector3f const &positionTarget,
	Eigen::Vector3f const &velocityTarget,
	Eigen::Vector3f const &accelTarget)
	:
	posTgt(positionTarget),
	velTgt(velocityTarget),
	accelTgt(accelTarget)
{}

Eigen::Vector3f TrajectoryConstantState::pos(DWORD sys_time_ms) {
	return posTgt;
}

Eigen::Vector3f TrajectoryConstantState::vel(DWORD sys_time_ms) {
	return velTgt;
}

Eigen::Vector3f TrajectoryConstantState::accel(DWORD sys_time_ms) {
	return accelTgt;
}

std::shared_ptr<Trajectory> TrajectoryConstantState::Create(
	const Eigen::Vector3f &positionTarget,
	const Eigen::Vector3f &velocityTarget,
	const Eigen::Vector3f &accelTarget
) {
	return std::make_shared<TrajectoryConstantState>(
		positionTarget,
		velocityTarget,
		accelTarget
		);
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
	return std::make_shared<TrajectoryBangBang>(
		timeTotal,
		sys_time_init,
		posInit,
		posEnd
		);
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

std::shared_ptr<Trajectory> TrajectoryCircle::Create(
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

std::shared_ptr<Trajectory> TrajectorySinusoid::Create(
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

std::shared_ptr<Trajectory> TrajectoryInfShape::Create(
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

std::shared_ptr<Trajectory> TrajectoryHeart::Create(
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

std::shared_ptr<Trajectory> TrajectoryBangbangWithDrag::Create(
	float force,
	float radius,
	DWORD sys_time_init,
	const Eigen::Vector3f& posInit,
	const Eigen::Vector3f& posEnd
) {
	return std::make_shared<TrajectoryBangbangWithDrag>(force, radius, sys_time_init, posInit, posEnd);
}

float TrajectoryBangbangWithDrag::terminal_velocity() {
	return 36914.f * std::sqrt(_force) / _radius;
}

float TrajectoryBangbangWithDrag::mu() {
	return 0.15f / _radius;
}

float TrajectoryBangbangWithDrag::beta() {
	float dist = (_posEnd - _posInit).norm();
	return std::sqrtf((std::expf(2 * mu() * dist) - 1 )/(std::expf(2 * mu() * dist) + 1));
}

float TrajectoryBangbangWithDrag::time_to_accel() {
	return std::logf((1 + beta()) / (1 - beta())) / 2.0f / mu() / terminal_velocity();
}

float TrajectoryBangbangWithDrag::time_to_decel() {
	return std::atan(beta()) / mu() / terminal_velocity();
}

float TrajectoryBangbangWithDrag::dist_to_accel() {
	auto dist = (_posEnd - _posInit).norm();
	return 0.5f / mu() * std::logf(0.5f * (std::expf(2.f * mu() * dist) + 1.f));
}

Eigen::Vector3f TrajectoryBangbangWithDrag::pos(DWORD time_ms) {
	auto vt = terminal_velocity();
	if (time_ms < _sys_time_init) {
		return _posInit;
	}
	float dt = (time_ms - _sys_time_init) / 1000.f;
	float dist = 0;
	if (dt < time_to_accel()) {
		dist = -vt * dt + 1.0f / mu() * std::logf((std::expf(2.f * mu() * vt * dt) + 1) / 2.0f);
	}
	else if (dt <= time_to_accel() + time_to_decel()) {
		float time_to_go = time_to_accel() + time_to_decel() - dt;
		dist = (_posEnd - _posInit).norm() + std::logf(std::abs(std::cosf(mu() * vt * time_to_go))) / mu();
	}
	else {
		dist = (_posEnd - _posInit).norm();
	}
	return dist * (_posEnd - _posInit).normalized() + _posInit;
}

Eigen::Vector3f TrajectoryBangbangWithDrag::vel(DWORD time_ms) {
	if (time_ms < _sys_time_init) {
		return Eigen::Vector3f::Zero();
	}
	float dt = (time_ms - _sys_time_init) / 1000.f;
	float speed = 0;
	if (dt < time_to_accel()) {
		speed = terminal_velocity() * (1 - 2.f / (expf(2 * mu() * terminal_velocity() * dt) + 1.f));
	}
	else if (dt < time_to_accel() + time_to_decel()) {
		auto time_to_go = time_to_accel() + time_to_decel() - dt;
		speed = terminal_velocity() * std::tanf(mu() * terminal_velocity() * time_to_go);
	}
	return speed * (_posEnd - _posInit).normalized();
}

Eigen::Vector3f TrajectoryBangbangWithDrag::accel(DWORD time_ms) {
	auto dt = (time_ms - _sys_time_init) / 1000.f;
	if (dt >= time_to_accel() + time_to_decel()) {
		return Eigen::Vector3f::Zero();
	}
	auto mass = static_cast<float>(1.168e-9f * 4.0f * float(M_PI) * _radius * _radius * _radius / 3.0f);
	auto a = _force / mass;
	if (dt > time_to_accel()) {
		a *= -1;
	}
	return a * (_posEnd - _posInit).normalized();
}