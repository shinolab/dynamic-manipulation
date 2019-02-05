#include "odcs.hpp"
#include "profile.hpp"
#include "arfModel.hpp"
#include <Eigen/Dense>
#include <Windows.h>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <memory>


using namespace boost::numeric::odeint;

Eigen::Vector3f profileUniAccel::posTgt(float const &time)
{
	DWORD dt = time - timeInit;
	return  posInit + 0.5 * dt * dt * accel;
}

Eigen::Vector3f profileUniAccel::velTgt(float const &time)
{
	return velInit + (time - timeInit) * accel;
}

Eigen::Vector3f profileUniAccel::accelTgt(float const &time)
{
	return accel;
}

Eigen::Vector3f profileBangBang::posTgt(float const &time)
{
	if (time < timeInit) return posInit;
	if (time > timeInit + timeTotal) return posEnd;
	float dt = time - timeInit;
	return (2.0f * dt < timeTotal) ?  0.5f * dt * dt * accelTgt(time) + posInit :  0.5f * (dt - timeTotal) * (dt - timeTotal) * accelTgt(time) + posEnd;
}

Eigen::Vector3f profileBangBang::velTgt(float const &time)
{
	if (time < timeInit || time >(timeInit + timeTotal)) return Eigen::Vector3f::Zero();
	float dt = time - timeInit;
	return 2.0f * dt < timeTotal ? accelTgt(time) * dt : accelTgt(time) * (dt - timeTotal);
}

Eigen::Vector3f profileBangBang::accelTgt(float const &time)
{
	if (time < timeInit || time >(timeInit + timeTotal)) return Eigen::Vector3f::Zero();
	return 	2.0f * (time - timeInit) < timeTotal ? 4.0f * (posEnd - posInit) / timeTotal / timeTotal :  4.0f * (posInit - posEnd) / timeTotal / timeTotal;
}

profileMaxVerticalVelocity::profileMaxVerticalVelocity(float const &duty_limit)
{
	this->duty_limit = duty_limit;
}

Eigen::Vector3f profileMaxVerticalVelocity::posTgt(float const &z)
{
	return Eigen::Vector3f(0.0f, 0.0f, z);
}

Eigen::Vector3f profileMaxVerticalVelocity::velTgt(float const &z)
{
	return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
}

Eigen::Vector3f profileMaxVerticalVelocity::accelTgt(float const &height)
{
	float z = height / 1000.0f;
	float forceMax = ((1.8455e-1f * z + 1.4013f) * z - 8.228f) * z +10.532;
	return duty_limit * forceMax * Eigen::Vector3f::UnitZ();
}

profileMaxAccel::profileMaxAccel(Eigen::Vector3f const &positionTerminal,
	Eigen::Vector3f const &velocityTerminal,
	Eigen::VectorXf const &dutyLimit,
	std::shared_ptr<ocs> const ocsPtr,
	FloatingObjectPtr objPtr,
	float dt):sys(velocityTerminal.normalized(), dutyLimit, objPtr, ocsPtr)
{
	state_type state{ positionTerminal, velocityTerminal };
	runge_kutta4<state_type, float> rk4;
	const int step_max = 100;
	for (int i_step = 0; i_step < step_max; i_step++)
	{
		float t = -dt * i_step;
		pathTime.push_back(t);
		pathPos.push_back(state[0]);
		pathVel.push_back(state[1]);
		rk4.do_step(sys, state, t, -dt);
		if (state[1].dot(velocityTerminal.normalized()) < 0.f){
			break;
		}
	}
}

void profileMaxAccel::sys::operator()(state_type const &x, state_type &dxdt, float const)
{
	dxdt[0] = x[1];
	Eigen::MatrixXf posRel = x[0].replicate(1, ocsPtr->centersAUTD.cols()) - ocsPtr->centersAUTD;
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
	std::cout << (ocsPtr->arfModelPtr->arf(posRel, ocsPtr->eulerAnglesAUTD) * duty).transpose() << std::endl;
	dxdt[1] = (ocsPtr->arfModelPtr->arf(posRel, ocsPtr->eulerAnglesAUTD) * duty - 0.1e-3 * Eigen::Vector3f(0.f, 0.f, 9.80665e3f)) / 5.5e-3f ;
}

Eigen::Vector3f profileMaxAccel::posTgt(float const &t)
{
	return Eigen::Vector3f::Zero();
}

Eigen::Vector3f profileMaxAccel::velTgt(float const &t)
{
	return Eigen::Vector3f::Zero();
}

Eigen::Vector3f profileMaxAccel::accelTgt(float const &t)
{
	return Eigen::Vector3f::Zero();
}

Eigen::Vector3f profileMaxAccel::posInit() {
	return *pathPos.rbegin();
}