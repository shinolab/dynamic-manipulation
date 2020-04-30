#include "odcs.hpp"
#include "arfModel.hpp"
#include <Eigen/Dense>
#include <Windows.h>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <memory>

using namespace boost::numeric::odeint;
using namespace dynaman;

TrajectoryConstantState::TrajectoryConstantState(Eigen::Vector3f const &positionTarget,
	Eigen::Vector3f const &velocityTarget,
	Eigen::Vector3f const &accelTarget) :posTgt(positionTarget), velTgt(velocityTarget), accelTgt(accelTarget) {}

Eigen::Vector3f TrajectoryConstantState::pos(float const &time) {
	return posTgt;
}

Eigen::Vector3f TrajectoryConstantState::vel(float const &time) {
	return velTgt;
}

Eigen::Vector3f TrajectoryConstantState::accel(float const &time) {
	return accelTgt;
}

std::shared_ptr<Trajectory> TrajectoryConstantState::Create(Eigen::Vector3f const &positionTarget,
	Eigen::Vector3f const &velocityTarget,
	Eigen::Vector3f const &accelTarget) {
	return std::shared_ptr<Trajectory>(new TrajectoryConstantState(positionTarget, velocityTarget, accelTarget));
}

Eigen::Vector3f TrajectoryBangBang::pos(float const &time)
{
	if (time < timeInit) return posInit;
	if (time > timeInit + timeTotal) return posEnd;
	float dt = time - timeInit;
	return (2.0f * dt < timeTotal) ? 0.5f * dt * dt * accel(time) + posInit : 0.5f * (dt - timeTotal) * (dt - timeTotal) * accel(time) + posEnd;
}

Eigen::Vector3f TrajectoryBangBang::vel(float const &time)
{
	if (time < timeInit || time >(timeInit + timeTotal)) return Eigen::Vector3f::Zero();
	float dt = time - timeInit;
	return 2.0f * dt < timeTotal ? accel(time) * dt : accel(time) * (dt - timeTotal);
}

Eigen::Vector3f TrajectoryBangBang::accel(float const &time)
{
	if (time < timeInit || time >(timeInit + timeTotal)) return Eigen::Vector3f::Zero();
	return 	2.0f * (time - timeInit) < timeTotal ? 4.0f * (posEnd - posInit) / timeTotal / timeTotal : 4.0f * (posInit - posEnd) / timeTotal / timeTotal;
}

std::shared_ptr<Trajectory> TrajectoryBangBang::Create(float const &timeTotal,
	float const &timeInit,
	Eigen::Vector3f const &posInit,
	Eigen::Vector3f const &posEnd) {
	return std::shared_ptr<Trajectory>(new TrajectoryBangBang(timeTotal, timeInit, posInit, posEnd));
}

Eigen::Vector3f TrajectoryBang::pos(float const &time)
{
	if (time < timeInit || time > timeInit + 2 * timeToGo) return posInit;
	float dt = time - timeInit;
	if (dt < timeToGo) {
		return 0.5f * dt * dt * accel(time) + posInit;
	}
	else {
		return posInit + 0.5f * accel(time) * (2.f * timeToGo - dt) * (2.f * timeToGo - dt);
	}
}

Eigen::Vector3f TrajectoryBang::vel(float const &time)
{
	if (time < timeInit || time > timeInit + 2 * timeToGo) return Eigen::Vector3f::Zero();
	float dt = time - timeInit;
	return dt < timeToGo ? accel(time) * dt : accel(time) * (2 * timeToGo - dt);
}

Eigen::Vector3f TrajectoryBang::accel(float const &time)
{
	if (time < timeInit || time > timeInit + 2 * timeToGo) return Eigen::Vector3f::Zero();
	float dt = time - timeInit;
	return 	dt < timeToGo ? 2.0f * (posEnd - posInit) / timeToGo / timeToGo : 2.0f * (posInit - posEnd) / timeToGo / timeToGo;
}

std::shared_ptr<Trajectory> TrajectoryBang::Create(float const &timeToGo,
	float const &timeInit,
	Eigen::Vector3f const &posInit,
	Eigen::Vector3f const &posEnd) {
	return std::shared_ptr<Trajectory>(new TrajectoryBang(timeToGo, timeInit, posInit, posEnd));
}

TrajectoryMaxAccel::TrajectoryMaxAccel(Eigen::Vector3f const &positionTerminal,
	Eigen::Vector3f const &velocityTerminal,
	float terminalTime,
	Eigen::VectorXf const &dutyLimit,
	std::shared_ptr<ocs> const ocsPtr,
	FloatingObjectPtr objPtr,
	float dt) :sys(velocityTerminal.normalized(), dutyLimit, objPtr, ocsPtr)
{
	state_type state{ positionTerminal, velocityTerminal };
	runge_kutta4<state_type, float> rk4;
	const int step_max = 100;
	for (int i_step = 0; i_step < step_max; i_step++)
	{
		float t = -dt * i_step + terminalTime;
		pathTime.push_back(t);
		pathPos.push_back(state[0]);
		pathVel.push_back(state[1]);
		rk4.do_step(sys, state, t, -dt);
		if (state[1].dot(velocityTerminal.normalized()) < 0.f) {
			break;
		}
	}
}

void TrajectoryMaxAccel::sys::operator()(state_type const &x, state_type &dxdt, float const)
{
	dxdt[0] = x[1];
	Eigen::MatrixXf posRel = x[0].replicate(1, ocsPtr->CentersAUTD().cols()) - ocsPtr->CentersAUTD();
	Eigen::Vector3f constDirection1 = direction.cross(Eigen::Vector3f::UnitX());
	if (constDirection1.norm() < 1.0e-6) // in case that velocity direction is parallel to x-axis
	{
		constDirection1 = direction.cross(Eigen::Vector3f::UnitY());
	}
	constDirection1.normalize();
	Eigen::Vector3f constDirection2 = constDirection1.cross(direction);
	Eigen::MatrixXf constDirections(3, 2);
	constDirections << constDirection1, constDirection2;
	float force;
	Eigen::VectorXf duty = ocsPtr->FindDutyMaximizeForce(direction, constDirections, x[0], dutyLimit, force);
	//std::cout << (ocsPtr->arfModelPtr->arf(posRel, ocsPtr->eulerAnglesAUTD) * duty).transpose() << std::endl;
	dxdt[1] = (ocsPtr->arfModelPtr->arf(posRel, ocsPtr->eulerAnglesAUTD) * duty - 0.1e-3 * Eigen::Vector3f(0.f, 0.f, 9.80665e3f)) / 5.5e-3f;
}

Eigen::Vector3f TrajectoryMaxAccel::pos(float const &t)
{
	if (t > *pathTime.begin()) {
		return *pathPos.begin();
	}
	auto itrTime = std::find_if(pathTime.begin(), pathTime.end(), [&t](float timeStamp) { return timeStamp < t; });
	if (itrTime == pathTime.end()) {
		return *pathPos.rbegin();
	}
	else {
		int index = std::distance(pathTime.begin(), itrTime);
		return ((pathTime[index - 1] - t) * pathPos[index] + (t - pathTime[index]) * pathPos[index - 1]) / (pathTime[index - 1] - pathTime[index]);
	}
}

Eigen::Vector3f TrajectoryMaxAccel::vel(float const &t)
{
	if (t > *pathTime.begin()) {
		return *pathVel.begin();
	}
	auto itrTime = std::find_if(pathTime.begin(), pathTime.end(), [&t](float timeStamp) {return timeStamp < t; });
	if (itrTime == pathTime.end()) {
		return *pathVel.rbegin();
	}
	else {
		int index = std::distance(pathTime.begin(), itrTime);
		return ((pathTime[index - 1] - t) * pathVel[index] + (t - pathTime[index]) * pathVel[index - 1]) / (pathTime[index - 1] - pathTime[index]);
	}
}

Eigen::Vector3f TrajectoryMaxAccel::accel(float const &t)
{
	Eigen::Vector3f pt = pos(t);
	Eigen::Vector3f vt = vel(t);
	Eigen::MatrixXf posRel = pt.replicate(1, sys.ocsPtr->CentersAUTD().cols()) - sys.ocsPtr->CentersAUTD();
	float force;
	return sys.ocsPtr->arfModelPtr->arf(posRel, sys.ocsPtr->eulerAnglesAUTD)
		*sys.ocsPtr->FindDutyMaximizeForce(vt.normalized(), sys.direction, pt, sys.dutyLimit, force)
		/ sys.objPtr->totalMass();
}

Eigen::Vector3f TrajectoryMaxAccel::posInit() {
	return *pathPos.rbegin();
}

TrajectoryCircle::TrajectoryCircle(
	const Eigen::Vector3f& center,
	float radius,
	float inclination,
	float raan,
	float period,
	float phaseInit,
	float timeInit)
	:_center(center),
	_radius(radius),
	_inclination(inclination),
	_raan(raan),
	_omega(2 * M_PI / period),
	_phaseInit(phaseInit),
	_timeInit(timeInit) {}

std::shared_ptr<TrajectoryCircle> TrajectoryCircle::Create(
	const Eigen::Vector3f& center,
	float radius,
	float inclination,
	float raan,
	float period,
	float phaseInit,
	float timeInit) {
	return std::make_shared<TrajectoryCircle>(center, radius, inclination, raan, period, phaseInit, timeInit);
}

float TrajectoryCircle::Radius() {
	return _radius;
}

float TrajectoryCircle::Phase(float t) {
	return _phaseInit + (t - _timeInit) * _omega;
}

Eigen::Vector3f TrajectoryCircle::pos(float const &t) {
	return Eigen::AngleAxisf(_raan, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(_inclination, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(Phase(t), Eigen::Vector3f::UnitZ())
		* (_radius * Eigen::Vector3f::UnitX())
		+ _center;
}

Eigen::Vector3f TrajectoryCircle::vel(float const &t) {
	return Eigen::AngleAxisf(_raan, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(_inclination, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(Phase(t), Eigen::Vector3f::UnitZ())
		* (_omega * _radius * Eigen::Vector3f::UnitY());
}

Eigen::Vector3f TrajectoryCircle::accel(float const &t) {
	return Eigen::AngleAxisf(_raan, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(_inclination, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(Phase(t), Eigen::Vector3f::UnitZ())
		* (- _omega * _omega * _radius * Eigen::Vector3f::UnitX());
}

TrajectorySinusoid::TrajectorySinusoid(
	const Eigen::Vector3f& direction,
	float amplitude,
	float period,
	const Eigen::Vector3f& center,
	float timeInit)
	:_direction(direction),
	_amplitude(amplitude),
	_period(period),
	_center(center),
	_timeInit(timeInit) {}

std::shared_ptr<TrajectorySinusoid> TrajectorySinusoid::Create(
	const Eigen::Vector3f& direction,
	float amplitude,
	float period,
	const Eigen::Vector3f& center,
	float timeInit
) {
	return std::make_shared<TrajectorySinusoid>(direction, amplitude, period, center, timeInit);
}

Eigen::Vector3f TrajectorySinusoid::pos(const float& time) {
	return _center + _direction * _amplitude * sinf((Phase(time)));
}

Eigen::Vector3f TrajectorySinusoid::vel(const float& time) {
	return _direction * _amplitude * Omega(time) * cosf(Phase(time));
}

Eigen::Vector3f TrajectorySinusoid::accel(const float& time) {
	return -_direction * _amplitude * Omega(time) * Omega(time) * sinf(Phase(time));
}

float TrajectorySinusoid::Phase(float const& time) {
	return fmodf(time - _timeInit, _period) / _period * M_PI * 2.0f;
}

float TrajectorySinusoid::Omega(float const& time) {
	return 2.0f * M_PI / _period;
}