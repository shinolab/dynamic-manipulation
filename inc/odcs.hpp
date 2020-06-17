#ifndef _ODCS_HPP_
#define _ODCS_HPP_

#include "autd3.hpp"
#include <opencv2/core.hpp>
#include "arfModel.hpp"
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
	class PositionSensor;
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
			Eigen::Vector3f const &lowerbound,
			Eigen::Vector3f const &upperbound,
			float _additionalMass = 0.1e-3f,
			float radius = 90.f);

		float sphereMass(); //return a mass equivalent to an air of the volume of the sphere
		float AdditionalMass();
		float Radius();
		float totalMass();

		void updateStates(DWORD determinationTime, Eigen::Vector3f &positionNew);
		void updateStates(DWORD determinationTime, Eigen::Vector3f &positionNew, Eigen::Vector3f &velocitynew);
		void resetIntegral();

		void SetTrajectory(std::shared_ptr<Trajectory> newTrajectoryPtr);
		void updateStatesTarget(Eigen::Vector3f &_positionTarget,
			Eigen::Vector3f &_velocityTarget = Eigen::Vector3f(0, 0, 0),
			Eigen::Vector3f &_accelTarget = Eigen::Vector3f(0, 0, 0));

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

	public:
		int Initialize();
		void Close();
		int AddDevice(
			Eigen::Vector3f const &position,
			Eigen::Vector3f const &eulerAngles,
			int groupid = 0);
		Eigen::MatrixXf CentersAUTD();
		Eigen::MatrixXf DirectionsAUTD();

		void SetArfModel(std::unique_ptr<arfModelLinearBase> arfModelPtr);
		void SetGain(Eigen::Vector3f const &gainP, Eigen::Vector3f const &gainD, Eigen::Vector3f const &gainI);

		Eigen::VectorXf FindDutySVD(FloatingObjectPtr objPtr);
		Eigen::VectorXf FindDutyQPCGAL(Eigen::Vector3f const &force, Eigen::Vector3f const &position);
		Eigen::VectorXf FindDutyQpMultiplex(
			const Eigen::Vector3f& force,
			const Eigen::Vector3f& position,
			float lambda
		);
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
		int AddDevice(Eigen::Vector3f const &position, Eigen::Vector3f const &eulerAngles, int groupId = 0);
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
		virtual Eigen::Vector3f pos(DWORD time = timeGetTime()) = 0;
		virtual Eigen::Vector3f vel(DWORD time = timeGetTime()) = 0;
		virtual Eigen::Vector3f accel(DWORD time = timeGetTime()) = 0;
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
		Eigen::Vector3f pos(DWORD time = timeGetTime()) override;
		Eigen::Vector3f vel(DWORD time = timeGetTime()) override;
		Eigen::Vector3f accel(DWORD time = timeGetTime()) override;
		static std::shared_ptr<Trajectory> Create(Eigen::Vector3f const &positionTarget,
			Eigen::Vector3f const &velocityTarget,
			Eigen::Vector3f const &accelTarget);
	};

	class TrajectoryBangBang : public Trajectory
	{
	private:
		float _timeTotal;
		DWORD _sys_time_init;
		Eigen::Vector3f _posInit;
		Eigen::Vector3f _posEnd;
		Eigen::Vector3f _velInit;
		Eigen::Vector3f _velEnd;

	public:
		TrajectoryBangBang(float timeTotal,
			DWORD sys_time_init,
			Eigen::Vector3f const &posInit,
			Eigen::Vector3f const &posEnd)
			:_timeTotal(timeTotal),
			_sys_time_init(sys_time_init),
			_posInit(posInit),
			_posEnd(posEnd){}

		Eigen::Vector3f pos(DWORD time) override;
		Eigen::Vector3f vel(DWORD time) override;
		Eigen::Vector3f accel(DWORD time) override;
		static std::shared_ptr<Trajectory> Create(float timeTotal,
			DWORD sys_time_init,
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
		Eigen::Vector3f pos(DWORD const time = timeGetTime()) override;
		Eigen::Vector3f vel(DWORD const time = timeGetTime()) override;
		Eigen::Vector3f accel(DWORD const time = timeGetTime()) override;
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
		Eigen::Vector3f pos(DWORD const time = timeGetTime())override;
		Eigen::Vector3f vel(DWORD const time = timeGetTime()) override;
		Eigen::Vector3f accel(DWORD const time = timeGetTime()) override;
		Eigen::Vector3f posInit();
	};

	class TrajectoryCircle : public Trajectory {
		Eigen::Vector3f _center;
		float _radius;
		float _inclination;
		float _raan;
		float _omega;
		float _phaseInit;
		float _sys_time_init;

	public:
		TrajectoryCircle(
			const Eigen::Vector3f& _center,
			float _radius,
			float _inclination,
			float _raan,
			float _period,
			float _phaseInit,
			DWORD _sys_time_init
		);
		static std::shared_ptr<TrajectoryCircle> Create(
			const Eigen::Vector3f& center,
			float radius,
			float inclination,
			float raan,
			float period,
			float phaseInit,
			DWORD sys_time_init
		);
		Eigen::Vector3f pos(DWORD sys_time = timeGetTime()) override;
		Eigen::Vector3f vel(DWORD sys_time = timeGetTime()) override;
		Eigen::Vector3f accel(DWORD sys_time = timeGetTime()) override;
		float Radius();
		float Phase(DWORD sys_time);
	};

	class TrajectorySinusoid : public Trajectory {
		Eigen::Vector3f _direction;
		float _amplitude;
		float _period;
		Eigen::Vector3f _center;
		DWORD _sys_time_init;

	public:
		TrajectorySinusoid(
			const Eigen::Vector3f& direction,
			float amplitude,
			float period,
			const Eigen::Vector3f& center,
			DWORD sys_time_init
		);

		static std::shared_ptr<TrajectorySinusoid> Create(
			const Eigen::Vector3f& direction,
			float amplitude,
			float period,
			const Eigen::Vector3f& center,
			DWORD sys_time_init
		);
		
		Eigen::Vector3f pos(DWORD time = timeGetTime()) override;
		Eigen::Vector3f vel(DWORD time = timeGetTime()) override;
		Eigen::Vector3f accel(DWORD time = timeGetTime()) override;
		float Phase(DWORD sys_time);
		float Omega(DWORD sys_time);
	};

	class TrajectoryInf: public Trajectory {
		Eigen::Vector3f _center;
		float _height;
		float _width;
		float _omega;
		DWORD _sys_time_init;
	public:
		TrajectoryInf(
			const Eigen::Vector3f& center,
			float height,
			float width,
			float period,
			DWORD sys_time_init
		);

		~TrajectoryInf() = default;

		std::shared_ptr<TrajectoryInf> Create(
			const Eigen::Vector3f& center,
			float height,
			float width,
			float period,
			DWORD sys_time_init);

		Eigen::Vector3f pos(DWORD sys_time) override;

		Eigen::Vector3f vel(DWORD sys_time) override;

		Eigen::Vector3f accel(DWORD sys_time) override;

		float Phase(DWORD sys_time);

	};


}

#endif
