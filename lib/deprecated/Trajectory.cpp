#include "odcs.hpp"
#include "arfModel.hpp"
#include <Eigen/Dense>
#include <Windows.h>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <memory>

using namespace boost::numeric::odeint;

Eigen::Vector3f TrajectoryBang::pos(float const& time)
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

Eigen::Vector3f TrajectoryBang::vel(float const& time)
{
	if (time < timeInit || time > timeInit + 2 * timeToGo) return Eigen::Vector3f::Zero();
	float dt = time - timeInit;
	return dt < timeToGo ? accel(time) * dt : accel(time) * (2 * timeToGo - dt);
}

Eigen::Vector3f TrajectoryBang::accel(float const& time)
{
	if (time < timeInit || time > timeInit + 2 * timeToGo) return Eigen::Vector3f::Zero();
	float dt = time - timeInit;
	return 	dt < timeToGo ? 2.0f * (posEnd - posInit) / timeToGo / timeToGo : 2.0f * (posInit - posEnd) / timeToGo / timeToGo;
}

std::shared_ptr<Trajectory> TrajectoryBang::Create(float const& timeToGo,
	float const& timeInit,
	Eigen::Vector3f const& posInit,
	Eigen::Vector3f const& posEnd) {
	return std::shared_ptr<Trajectory>(new TrajectoryBang(timeToGo, timeInit, posInit, posEnd));
}

TrajectoryMaxAccel::TrajectoryMaxAccel(Eigen::Vector3f const& positionTerminal,
	Eigen::Vector3f const& velocityTerminal,
	float terminalTime,
	Eigen::VectorXf const& dutyLimit,
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

void TrajectoryMaxAccel::sys::operator()(state_type const& x, state_type& dxdt, float const)
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

Eigen::Vector3f TrajectoryMaxAccel::pos(float const& t)
{
	if (t > * pathTime.begin()) {
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

Eigen::Vector3f TrajectoryMaxAccel::vel(float const& t)
{
	if (t > * pathTime.begin()) {
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

Eigen::Vector3f TrajectoryMaxAccel::accel(float const& t)
{
	Eigen::Vector3f pt = pos(t);
	Eigen::Vector3f vt = vel(t);
	Eigen::MatrixXf posRel = pt.replicate(1, sys.ocsPtr->CentersAUTD().cols()) - sys.ocsPtr->CentersAUTD();
	float force;
	return sys.ocsPtr->arfModelPtr->arf(posRel, sys.ocsPtr->eulerAnglesAUTD)
		* sys.ocsPtr->FindDutyMaximizeForce(vt.normalized(), sys.direction, pt, sys.dutyLimit, force)
		/ sys.objPtr->totalMass();
}

Eigen::Vector3f TrajectoryMaxAccel::posInit() {
	return *pathPos.rbegin();
}