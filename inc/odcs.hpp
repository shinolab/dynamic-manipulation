#ifndef _ODCS_HPP_
#define _ODCS_HPP_

#include "Trajectory.hpp"
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
		FloatingObject(Eigen::Vector3f const& _positionTarget,
			Eigen::Vector3f const& lowerbound,
			Eigen::Vector3f const& upperbound,
			float _additionalMass = 0.1e-3f,
			float _radius = 90.f);

		static FloatingObjectPtr Create(Eigen::Vector3f const& posTgt,
			Eigen::Vector3f const& lowerbound,
			Eigen::Vector3f const& upperbound,
			float _additionalMass = 0.1e-3f,
			float radius = 90.f);

		float sphereMass(); //return a mass equivalent to an air of the volume of the sphere
		float AdditionalMass();
		float Radius();
		float totalMass();

		void updateStates(DWORD determinationTime, Eigen::Vector3f& positionNew);
		void updateStates(DWORD determinationTime, Eigen::Vector3f& positionNew, Eigen::Vector3f& velocitynew);
		void resetIntegral();

		void SetTrajectory(std::shared_ptr<Trajectory> newTrajectoryPtr);
		void updateStatesTarget(Eigen::Vector3f& _positionTarget,
			Eigen::Vector3f& _velocityTarget = Eigen::Vector3f(0, 0, 0),
			Eigen::Vector3f& _accelTarget = Eigen::Vector3f(0, 0, 0));

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
		virtual bool observe(DWORD& time, Eigen::Vector3f& pos, FloatingObjectPtr objPtr) = 0;
	};

	class single_actuator {
	public:
		single_actuator(autd::Controller& autd) :_autd(autd) {};
		virtual ~single_actuator() {};
		virtual void actuate(FloatingObjectPtr objPtr) = 0;
	protected:
		autd::Controller& _autd;
	};

	class multiple_actuator {
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
			Eigen::Vector3f const& position,
			Eigen::Vector3f const& eulerAngles,
			int groupid = 0);
		Eigen::MatrixXf CentersAUTD();
		Eigen::MatrixXf DirectionsAUTD();

		void SetArfModel(std::unique_ptr<arfModelLinearBase> arfModelPtr);
		void SetGain(Eigen::Vector3f const& gainP, Eigen::Vector3f const& gainD, Eigen::Vector3f const& gainI);

		Eigen::VectorXf FindDutySVD(FloatingObjectPtr objPtr);
		Eigen::VectorXf FindDutyQPCGAL(Eigen::Vector3f const& force, Eigen::Vector3f const& position);
		Eigen::VectorXf FindDutyQpMultiplex(
			const Eigen::Vector3f& force,
			const Eigen::Vector3f& position,
			float lambda
		);
		Eigen::VectorXf FindDutySelectiveQP(Eigen::Vector3f const& force, Eigen::Vector3f const& position, float const threshold = 0.7071f);
		Eigen::VectorXf FindDutyMaximizeForce(Eigen::Vector3f const& direction,
			Eigen::MatrixXf const& constrainedDirections,
			Eigen::Vector3f const& position,
			Eigen::VectorXf const& duty_limit,
			float& force,
			Eigen::Vector3f const& force_offset = Eigen::Vector3f(0.f, 0.f, 0.f));

		Eigen::VectorXf FindDutyQPMulti(Eigen::Matrix3Xf const& forces, Eigen::Matrix3Xf const& positions, float const penalty = 0.01f);

		autd::GainPtr CreateBalanceGain(FloatingObjectPtr objPtr, int numObj = 1);
		std::vector<autd::GainPtr> CreateBalanceGainMulti(std::vector<FloatingObjectPtr> const& objPtr);
	};

	class odcs
	{
	public:
		odcs(PositionSensor& sensor);
		void Initialize();
		//std::shared_ptr<ods> Sensor();
		std::shared_ptr<ocs> Controller();
		int AddDevice(Eigen::Vector3f const& position, Eigen::Vector3f const& eulerAngles, int groupId = 0);
		void RegisterObject(FloatingObjectPtr objPtr);
		void SetSensor(PositionSensor& new_sensor);
		const FloatingObjectPtr GetFloatingObject(int i);
		void StartControl();
		void ControlLoop(std::vector<FloatingObjectPtr>& objPtrs, int loopPeriod);
		void Close();
		void DetermineStateKF(FloatingObjectPtr objPtr, const Eigen::Vector3f& observe, const DWORD determinationTime);
		PositionSensor& sensor;

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
}
#endif
