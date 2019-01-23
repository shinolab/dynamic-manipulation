#include <Eigen/Dense>
#include <Windows.h>

#pragma comment(lib, "winmm")

class profile
{
public:
	virtual Eigen::Vector3f posTgt(DWORD time) = 0;
	virtual Eigen::Vector3f velTgt(DWORD time) = 0;
	virtual Eigen::Vector3f accelTgt(DWORD time) = 0;
};

class profileUniAccel : profile
{
public:
	profileUniAccel(Eigen::Vector3f const &acceleration
		, DWORD const &timeEnd
		, DWORD const &timeInit
		, Eigen::Vector3f const &posInit
		, Eigen::Vector3f const &velInit)
	{
		this->timeInit = timeInit;
		this->accel = acceleration;
		this->posInit = posInit;
		this->velInit = velInit;
	}

	Eigen::Vector3f posTgt(DWORD time = timeGetTime());
	Eigen::Vector3f velTgt(DWORD time = timeGetTime());
	Eigen::Vector3f accelTgt(DWORD time = timeGetTime());

private:
	DWORD timeInit;
	DWORD timeEnd;
	Eigen::Vector3f accel;
	Eigen::Vector3f posInit;
	Eigen::Vector3f velInit;
};