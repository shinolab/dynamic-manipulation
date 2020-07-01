#ifndef _ODCS_HPP_
#define _ODCS_HPP_

#include "Trajectory.hpp"
#include "FloatingObject.hpp"
#include "tracker.hpp"
#include "autd3.hpp"
#include "arfModel.hpp"
#include "strategy.hpp"
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
		odcs(Tracker& tracker);
		void Initialize();
		//std::shared_ptr<ods> Sensor();
		std::shared_ptr<ocs> Controller();
		int AddDevice(Eigen::Vector3f const& position, Eigen::Vector3f const& eulerAngles, int groupId = 0);
		void RegisterObject(FloatingObjectPtr objPtr);
		void SetSensor(Tracker& new_tracker);
		const FloatingObjectPtr GetFloatingObject(int i);
		void StartControl();
		void ControlLoop(std::vector<FloatingObjectPtr>& objPtrs, int loopPeriod);
		void Close();
		void DetermineStateKF(FloatingObjectPtr objPtr, const Eigen::Vector3f& observe, const DWORD determinationTime);
		Tracker& m_tracker;

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
