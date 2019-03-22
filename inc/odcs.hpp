#ifndef _ODCS_H_
#define _ODCS_H_

#include "autd3.hpp"
#include "KinectApp.hpp"
#include <opencv2/core.hpp>
#include "arfModel.hpp"
//#include "engine.h"
#include <queue>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Dense>
#define NOMINMAX
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

#pragma comment (lib, "winmm")

class FloatingObject;
typedef std::shared_ptr<FloatingObject> FloatingObjectPtr;
class ods;
class ocs;
class Trajectory;

class FloatingObject {
public:
	Eigen::Vector3f getPosition();
	Eigen::Vector3f getVelocity();
	Eigen::Vector3f getIntegral();
	Eigen::Vector3f getPositionTarget();
	Eigen::Vector3f getVelocityTarget();
	Eigen::Vector3f getAccelTarget();
	Eigen::VectorXf getLatestInput();
	void setLatestInput(Eigen::VectorXf input);
	Eigen::VectorXf inputLatest;

	Eigen::MatrixXf covError;
	DWORD lastDeterminationTime;
	bool _isStable;
	bool isControlled;
	std::deque<Eigen::Vector3f> velocityBuffer;
	std::deque<float> dTBuffer;
	const int velocityBufferSize = 3;
	float radius = 90.0; // [mm]
	//const float mass = 5.4e-3; //[Kg]
	float speedLimit = 400; // [mm/s]
	float additionalMass = 0.1e-3;

private:
	Eigen::Vector3f position;
	Eigen::Vector3f velocity;
	Eigen::Vector3f integral;
	bool isTracked;
	std::mutex mtxState;
	std::mutex mtxTrack;
	std::shared_mutex mtxTrajectory;
	std::shared_ptr<Trajectory> trajectoryPtr;
public:
	FloatingObject(Eigen::Vector3f const &_positionTarget, float _additionalMass = 0.1e-3);
	static FloatingObjectPtr Create(Eigen::Vector3f const &posTgt, float _additionalMass = 0.1e-3);

	float sphereMass(); //return a mass equivalent to an air of the volume of the sphere
	float AdditionalMass();
	float Radius();
	float totalMass(); 

	void updateStates(DWORD determinationTime, Eigen::Vector3f &positionNew);
	void updateStates(DWORD determinationTime, Eigen::Vector3f &positionNew, Eigen::Vector3f &velocitynew);

	void SetTrajectory(std::shared_ptr<Trajectory> newTrajectoryPtr);
	void updateStatesTarget(Eigen::Vector3f &_positionTarget, Eigen::Vector3f &_velocityTarget, Eigen::Vector3f &_accelTarget = Eigen::Vector3f(0, 0, 0));

	bool isStable();
	bool isConverged(float tolPos, float tolVel);
	bool IsTracked();
	void SetTrackingStatus(bool _isTracked);
	Eigen::Vector3f averageVelocity();
};

class ods
{
public:
private:
	typedef Eigen::Matrix<float, 3, 8> Matrix38f;
	KinectApp kinectApp;
	Eigen::Vector3f positionKinect;
	Eigen::Matrix3f dcmGlobal2Kinect;
	Eigen::Matrix3f dcmKinect2Global; // x_kinect = attitudeKinect * x_global
	Eigen::Affine3f affineKinect2Global;
	Eigen::Matrix<float, 3, 2> workspace;
	std::vector<UINT16> backgroundDepth;

public:
	int Initialize();
	void SetSensorGeometry(Eigen::Vector3f const &position, Eigen::Vector3f const &eulerAngle);
	void SetWorkSpace(Eigen::Vector3f const &corner1, Eigen::Vector3f const &corner2);
	void CornersWorkspaceAll(Matrix38f &corners);
	void MaskWorkspace(cv::Mat &mask);
	float RangeWorkspace();
	Eigen::Affine3f getAffineKinect2Global() { return affineKinect2Global; }
	Eigen::Matrix3f getDcmGlobal2Kinect() { return dcmGlobal2Kinect; }
	Eigen::Matrix3f getDcmKinect2Global() { return dcmKinect2Global; }
	bool isInsideWorkSpace(const Eigen::Vector3f &pos);

	void DeterminePositionByHSV(FloatingObjectPtr objPtr, cv::Scalar lb, cv::Scalar ub);
	void DeterminePositionByBGR(FloatingObjectPtr objPtr, cv::Scalar lb, cv::Scalar ub);
	void DeterminePositionByDepth(FloatingObjectPtr objPtr, bool useROI);
	
	bool GetPositionByBGR(FloatingObjectPtr objPtr, Eigen::Vector3f &pos, cv::Scalar lb, cv::Scalar ub);
	bool GetPositionByHSV(FloatingObjectPtr objPtr, Eigen::Vector3f &pos, cv::Scalar lb, cv::Scalar ub);
	bool GetPositionByDepth(FloatingObjectPtr objPtr, Eigen::Vector3f &pos, bool useROI);

	HRESULT updateBackgroundDepth();
	bool findSphere(const cv::Mat src, cv::Point &center, float &radius);
};

class ocs
{
public:
	autd::Controller _autd;
	Eigen::MatrixXf positionsAUTD;
	Eigen::MatrixXf eulerAnglesAUTD;
	std::unique_ptr<arfModelLinearBase> arfModelPtr;
	void RegisterObject(FloatingObjectPtr objPtr);

private:
	Eigen::Vector3f gainP = -1.6 * Eigen::Vector3f::Ones();
	Eigen::Vector3f gainD = -4.0 * Eigen::Vector3f::Ones();
	Eigen::Vector3f gainI = -0.04 * Eigen::Vector3f::Ones();
	//Engine *ep;

public:
	int Initialize();

	void Close();

	int AddDevice(Eigen::Vector3f const &position, Eigen::Vector3f const &eulerAngles);

	Eigen::MatrixXf CentersAUTD();

	Eigen::MatrixXf DirectionsAUTD();

	void SetArfModel(std::unique_ptr<arfModelLinearBase> arfModelPtr);

	void SetGain(Eigen::Vector3f const &gainP, Eigen::Vector3f const &gainD, Eigen::Vector3f const &gainI);

	Eigen::VectorXf FindDutyQP(Eigen::Vector3f const &force, Eigen::Vector3f const &position);

	Eigen::VectorXf FindDutyQP(Eigen::Vector3f const &force, Eigen::Vector3f const &position, Eigen::VectorXf const &duty_forward);

	Eigen::VectorXf FindDutySVD(FloatingObjectPtr objPtr);

	Eigen::VectorXf FindDutyMaximizeForce(Eigen::Vector3f const &direction, 
		Eigen::MatrixXf const &constrainedDirections, 
		Eigen::Vector3f const &position,
		Eigen::VectorXf const &duty_limit,
		float &force,
		Eigen::Vector3f const &force_offset = Eigen::Vector3f(0.f, 0.f, 0.f));
	
	autd::GainPtr CreateGain(FloatingObjectPtr objPtr);

	//Legacy Module
	Eigen::VectorXf FindDutySI(FloatingObjectPtr objPtr);

	Eigen::VectorXf FindDutyQPEq(FloatingObjectPtr objPtr);
};

class odcs
{
public:
	void Initialize();
	int AddObject(Eigen::Vector3f const &positionTarget);
	int AddDevice(Eigen::Vector3f const &position, Eigen::Vector3f const &eulerAngles);
	void RegisterObject(FloatingObjectPtr objPtr);
	const FloatingObjectPtr GetFloatingObject(int i);
	void StartControl();
	void ControlLoop(std::vector<FloatingObjectPtr> &objPtrs, int loopPeriod);
	void Close();
	void DetermineStateKF(FloatingObjectPtr objPtr, const Eigen::Vector3f &observe, const DWORD determinationTime);
	std::shared_ptr<ods> odsPtr;
	std::shared_ptr<ocs> ocsPtr;
	std::thread thread_control;
private:
	std::vector<FloatingObjectPtr> objPtrs;
	std::shared_mutex mtxRunning;
	bool flagRunning = false;

public:
	bool isRunning();
};

class Trajectory
{
public:
	virtual ~Trajectory(){};
	virtual Eigen::Vector3f pos(float const &time = timeGetTime() / 1000.f) = 0;
	virtual Eigen::Vector3f vel(float const & time = timeGetTime() / 1000.f) = 0;
	virtual Eigen::Vector3f accel(float const & time = timeGetTime() / 1000.f) = 0;
};

class TrajectoryConstantState : public Trajectory
{
private:
	Eigen::Vector3f posTgt;
	Eigen::Vector3f velTgt;
	Eigen::Vector3f accelTgt;
public:
	TrajectoryConstantState(Eigen::Vector3f const &positionTarget,
		Eigen::Vector3f const &velocityTarget = Eigen::Vector3f::Constant(0.f),
		Eigen::Vector3f const &accelTarget = Eigen::Vector3f::Constant(0.f));
	Eigen::Vector3f pos(float const &time = timeGetTime() / 1000.f) override;
	Eigen::Vector3f vel(float const &time = timeGetTime() / 1000.f) override;
	Eigen::Vector3f accel(float const &time = timeGetTime() / 1000.f) override;
};

class TrajectoryBangBang : public Trajectory
{
private:
	float timeTotal;
	float timeInit;
	Eigen::Vector3f posInit;
	Eigen::Vector3f posEnd;
	Eigen::Vector3f velInit;
	Eigen::Vector3f velEnd;

public:
	TrajectoryBangBang(float const &timeTotal,
		float const &timeInit,
		Eigen::Vector3f const &posInit,
		Eigen::Vector3f const &posEnd)
	{
		this->timeInit = timeInit;
		this->timeTotal = timeTotal;
		this->posInit = posInit;
		this->posEnd = posEnd;
	}
	Eigen::Vector3f pos(float const &time = timeGetTime() / 1000.0f) override;
	Eigen::Vector3f vel(float const &time = timeGetTime() / 1000.0f) override;
	Eigen::Vector3f accel(float const &time = timeGetTime() / 1000.0f) override;
};

class TrajectoryMaxAccel : public Trajectory {
public:
	typedef std::vector<Eigen::Vector3f> state_type;
private:
	class sys {
	public:
		FloatingObjectPtr objPtr;
		std::shared_ptr<ocs> ocsPtr;
		Eigen::Vector3f const &direction;
		Eigen::VectorXf const &dutyLimit;
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
	TrajectoryMaxAccel(Eigen::Vector3f const &positionTerminal,
		Eigen::Vector3f const &velocityTerminal,
		float terminalTime,
		Eigen::VectorXf const &duty_limit,
		std::shared_ptr<ocs> ocsPtr,
		FloatingObjectPtr objPtr,
		float dt = 0.1f);
	Eigen::Vector3f pos(float const &time = timeGetTime() / 1000.f) override;
	Eigen::Vector3f vel(float const &time = timeGetTime() / 1000.f) override;
	Eigen::Vector3f accel(float const &time = timeGetTime() / 1000.f) override;
	Eigen::Vector3f posInit();
};

class TrajectoryCircle : public Trajectory {
	float radius;
	float omega;
	float timeInit;
	float phaseInit;
	Eigen::Vector3f center;
public:
	TrajectoryCircle(Eigen::Vector3f &center, float radius, float period, float timeInit, float phaseInit = 0.f) :radius(radius), omega(2 * M_PI / period), phaseInit(phaseInit), timeInit(timeInit), center(center) {}
	Eigen::Vector3f pos(float const &time = timeGetTime() / 1000.f) override;
	Eigen::Vector3f vel(float const &time = timeGetTime() / 1000.f) override;
	Eigen::Vector3f accel(float const &time = timeGetTime() / 1000.f) override;
};


#endif
