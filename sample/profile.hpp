#ifndef _ODCS_PROFILE_H_
#define _ODCS_PROFILE_H_

#include "odcs.hpp"
#include "arfModel.hpp"
#include "QPSolver.h"
#include <Eigen/Dense>
#include <Windows.h>
#include <vector>
#include <memory>

#pragma comment(lib, "winmm")

class profile
{
public:
	virtual Eigen::Vector3f posTgt(float const &time) = 0;
	virtual Eigen::Vector3f velTgt(float const & time) = 0;
	virtual Eigen::Vector3f accelTgt(float const & time) = 0;
};

class profileUniAccel : public profile
{
private:
	float timeInit;
	float timeEnd;
	Eigen::Vector3f accel;
	Eigen::Vector3f posInit;
	Eigen::Vector3f velInit;

public:
	profileUniAccel(Eigen::Vector3f const &acceleration
		, float const &timeEnd
		, float const &timeInit
		, Eigen::Vector3f const &posInit
		, Eigen::Vector3f const &velInit)
	{
		this->timeInit = timeInit;
		this->accel = acceleration;
		this->posInit = posInit;
		this->velInit = velInit;
	}

	Eigen::Vector3f posTgt(float const &time = timeGetTime() / 1000.0f) override;
	Eigen::Vector3f velTgt(float  const &time = timeGetTime() / 1000.0f) override;
	Eigen::Vector3f accelTgt(float const &time = timeGetTime() / 1000.0f) override;
};

class profileBangBang : public profile
{
private:
	float timeTotal;
	float timeInit;
	Eigen::Vector3f posInit;
	Eigen::Vector3f posEnd;
	Eigen::Vector3f velInit;
	Eigen::Vector3f velEnd;

public:
	profileBangBang(float const &timeTotal,
		float const &timeInit,
		Eigen::Vector3f const &posInit,
		Eigen::Vector3f const &posEnd)
	{
		this->timeInit = timeInit;
		this->timeTotal = timeTotal;
		this->posInit = posInit;
		this->posEnd = posEnd;
	}
	Eigen::Vector3f posTgt(float const &time = timeGetTime() / 1000.0f) override;
	Eigen::Vector3f velTgt(float const &time = timeGetTime() / 1000.0f) override;
	Eigen::Vector3f accelTgt(float const &time = timeGetTime() / 1000.0f) override;

};

class profileMaxVerticalVelocity : public profile
{
public:
	profileMaxVerticalVelocity(float const &duty_limit);
	Eigen::Vector3f posTgt(float const &z);
	Eigen::Vector3f velTgt(float const &z);
	Eigen::Vector3f accelTgt(float const &z);
private:
	float duty_limit;
};

class profileMaxAccel : public profile {
public:
	typedef std::vector<Eigen::Vector3f> state_type;
private:
	class sys{
	private:
		Eigen::Vector3f const &direction;
		Eigen::VectorXf const &dutyLimit;
		FloatingObjectPtr objPtr;
		std::shared_ptr<ocs> ocsPtr;
	public:
		sys(Eigen::Vector3f const &direction, Eigen::VectorXf const &dutyLimit, FloatingObjectPtr objPtr, std::shared_ptr<ocs> ocsPtr) : direction(direction), objPtr(objPtr), dutyLimit(dutyLimit), ocsPtr(ocsPtr) {};
		void operator()(state_type const &x, state_type &dxdt, float const t);
	};
	sys sys;
public:
	std::vector<float> pathTime;
	std::vector<Eigen::Vector3f> pathPos;
	std::vector<Eigen::Vector3f> pathVel;
	std::vector<Eigen::Vector3f> pathAccel;
	
public:
	profileMaxAccel(Eigen::Vector3f const &positionTerminal,
		Eigen::Vector3f const &velocityTerminal,
		Eigen::VectorXf const &duty_limit,
		std::shared_ptr<ocs> ocsPtr,
		FloatingObjectPtr objPtr,
		float dt = 0.1f);
	Eigen::Vector3f posTgt(float const &time = timeGetTime() / 1000.f) override;
	Eigen::Vector3f velTgt(float const &time = timeGetTime() / 1000.f) override;
	Eigen::Vector3f accelTgt(float const &time = timeGetTime() / 1000.f) override;
	Eigen::Vector3f posInit();
};

#endif