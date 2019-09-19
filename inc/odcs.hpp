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

namespace dynaman {
	class FloatingObject;
	typedef std::shared_ptr<FloatingObject> FloatingObjectPtr;
	class Sensor;
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
		const int velocityBufferSize;
		std::deque<Eigen::Vector3f> positionBuffer;
		std::deque<Eigen::Vector3f> velocityBuffer;
		std::deque<float> dTBuffer;
		float radius; // [mm]
		float additionalMass;

	private:
		Eigen::Vector3f position;
		Eigen::Vector3f velocity;
		Eigen::Vector3f integral;
		Eigen::Vector3f _upperbound;
		Eigen::Vector3f _lowerbound;
		bool isTracked;
		std::mutex mtxState;
		std::mutex mtxTrack;
		std::shared_mutex mtxTrajectory;
		std::shared_ptr<Trajectory> trajectoryPtr;

	public:
		FloatingObject(Eigen::Vector3f const &_positionTarget,
			Eigen::Vector3f const &lowerbound,
			Eigen::Vector3f const &upperbound,
			float _additionalMass = 0.1e-3f,
			float _radius = 90.f);

		static FloatingObjectPtr Create(Eigen::Vector3f const &posTgt,
			Eigen::Vector3f const &lpperbound,
			Eigen::Vector3f const &uowerbound,
			float _additionalMass = 0.1e-3f,
			float radius = 90.f);

		float sphereMass(); //return a mass equivalent to an air of the volume of the sphere
		float AdditionalMass();
		float Radius();
		float totalMass();

		void updateStates(DWORD determinationTime, Eigen::Vector3f &positionNew);
		void updateStates(DWORD determinationTime, Eigen::Vector3f &positionNew, Eigen::Vector3f &velocitynew);

		void SetTrajectory(std::shared_ptr<Trajectory> newTrajectoryPtr);
		void updateStatesTarget(Eigen::Vector3f &_positionTarget, Eigen::Vector3f &_velocityTarget, Eigen::Vector3f &_accelTarget = Eigen::Vector3f(0, 0, 0));

		bool isConverged(float tolPos, float tolVel);
		bool IsTracked();
		void SetTrackingStatus(bool _isTracked);
		Eigen::Vector3f averageVelocity();
		Eigen::Vector3f AveragePosition();
		Eigen::Vector3f lowerbound();
		Eigen::Vector3f upperbound();
	};

	class PositionSensor {
	public:
		virtual ~PositionSensor() {};
		virtual bool observe(DWORD &time, Eigen::Vector3f &pos, FloatingObjectPtr objPtr) = 0;
	};

	class single_actuator{
	public:
		single_actuator(autd::Controller& autd) :_autd(autd) {};
		virtual ~single_actuator() {};
		virtual void actuate(FloatingObjectPtr objPtr) = 0;
	protected:
		autd::Controller& _autd;
	};

	class multiple_actuator{
	public:
		multiple_actuator(autd::Controller& autd) :_autd(autd) {};
		virtual ~multiple_actuator() {};
		virtual void actuate(std::vector<FloatingObjectPtr> objPtrs) = 0;
	protected:
		autd::Controller& _autd;
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

		Eigen::VectorXf FindDutySVD(FloatingObjectPtr objPtr);
		Eigen::VectorXf FindDutyQPCGAL(Eigen::Vector3f const &force, Eigen::Vector3f const &position);
		Eigen::VectorXf FindDutySelectiveQP(Eigen::Vector3f const &force, Eigen::Vector3f const &position, float const threshold = 0.7071f);
		Eigen::VectorXf FindDutyMaximizeForce(Eigen::Vector3f const &direction,
			Eigen::MatrixXf const &constrainedDirections,
			Eigen::Vector3f const &position,
			Eigen::VectorXf const &duty_limit,
			float &force,
			Eigen::Vector3f const &force_offset = Eigen::Vector3f(0.f, 0.f, 0.f));

		Eigen::VectorXf FindDutyQPMulti(Eigen::Matrix3Xf const &forces, Eigen::Matrix3Xf const &positions, float const penalty = 0.01f);

		autd::GainPtr CreateBalanceGain(FloatingObjectPtr objPtr, int numObj = 1);
		std::vector<autd::GainPtr> CreateBalanceGainMulti(std::vector<FloatingObjectPtr> const &objPtr);
	};

	class odcs
	{
	public:
		odcs(PositionSensor& sensor);
		void Initialize();
		//std::shared_ptr<ods> Sensor();
		std::shared_ptr<ocs> Controller();
		int AddDevice(Eigen::Vector3f const &position, Eigen::Vector3f const &eulerAngles);
		void RegisterObject(FloatingObjectPtr objPtr);
		void SetSensor(PositionSensor &new_sensor);
		const FloatingObjectPtr GetFloatingObject(int i);
		void StartControl();
		void ControlLoop(std::vector<FloatingObjectPtr> &objPtrs, int loopPeriod);
		void Close();
		void DetermineStateKF(FloatingObjectPtr objPtr, const Eigen::Vector3f &observe, const DWORD determinationTime);
		PositionSensor &sensor;

		//std::shared_ptr<ods> odsPtr;
		std::shared_ptr<ocs> ocsPtr;
		std::thread thread_control;
		std::vector<FloatingObjectPtr> objPtrs;
	private:
		std::shared_mutex mtxRunning;
		bool flagRunning;

	public:
		bool isRunning();
	};

	class Trajectory
	{
	public:
		virtual ~Trajectory() {};
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
		static std::shared_ptr<Trajectory> Create(Eigen::Vector3f const &positionTarget,
			Eigen::Vector3f const &velocityTarget,
			Eigen::Vector3f const &accelTarget);
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
		static std::shared_ptr<Trajectory> Create(float const &timeTotal,
			float const &timeInit,
			Eigen::Vector3f const &posInit,
			Eigen::Vector3f const &posEnd);
	};

	class TrajectoryBang : public Trajectory {
	private:
	private:
		float timeToGo;
		float timeInit;
		Eigen::Vector3f posInit;
		Eigen::Vector3f posEnd;
	public:
		TrajectoryBang(float const &timeToGo,
			float const &timeInit,
			Eigen::Vector3f const &posInit,
			Eigen::Vector3f const &posEnd)
		{
			this->timeToGo = timeToGo;
			this->timeInit = timeInit;
			this->posInit = posInit;
			this->posEnd = posEnd;
		}
		Eigen::Vector3f pos(float const &time = timeGetTime() / 1000.0f) override;
		Eigen::Vector3f vel(float const &time = timeGetTime() / 1000.0f) override;
		Eigen::Vector3f accel(float const &time = timeGetTime() / 1000.0f) override;
		static std::shared_ptr<Trajectory> Create(float const &timeToGo,
			float const &timeInit,
			Eigen::Vector3f const &posInit,
			Eigen::Vector3f const &posEnd);
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

}

#endif
