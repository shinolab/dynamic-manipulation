#include <Eigen/Dense>
#include <Windows.h>

#pragma comment(lib, "winmm")

class profile
{
public:
	virtual Eigen::Vector3f posTgt(float const &time) = 0;
	virtual Eigen::Vector3f velTgt(float const & time) = 0;
	virtual Eigen::Vector3f accelTgt(float const & time) = 0;
};

class profileUniAccel : profile
{
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

	Eigen::Vector3f posTgt(float const &time = timeGetTime() / 1000.0f);
	Eigen::Vector3f velTgt(float  const &time = timeGetTime() / 1000.0f);
	Eigen::Vector3f accelTgt(float const &time = timeGetTime() / 1000.0f);

private:
	float timeInit;
	float timeEnd;
	Eigen::Vector3f accel;
	Eigen::Vector3f posInit;
	Eigen::Vector3f velInit;
};

class profileBangBang : profile
{
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
	Eigen::Vector3f posTgt(float const &time = timeGetTime() / 1000.0f);
	Eigen::Vector3f velTgt(float const &time = timeGetTime() / 1000.0f);
	Eigen::Vector3f accelTgt(float const &time = timeGetTime() / 1000.0f);

private:
	float timeTotal;
	float timeInit;
	Eigen::Vector3f posInit;
	Eigen::Vector3f posEnd;
	Eigen::Vector3f velInit;
	Eigen::Vector3f velEnd;
};

class profileMaxVerticalVelocity : profile
{
public:
	profileMaxVerticalVelocity(float const &duty_limit);
	Eigen::Vector3f posTgt(float const &z);
	Eigen::Vector3f velTgt(float const &z);
	Eigen::Vector3f accelTgt(float const &z);
private:
	float duty_limit;
};