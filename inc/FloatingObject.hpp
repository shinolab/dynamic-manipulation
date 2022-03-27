#pragma once

#include <memory>
#include <deque>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Geometry>
#include "Trajectory.hpp"

namespace dynaman {
	class FloatingObject;
	using FloatingObjectPtr = std::shared_ptr<FloatingObject>;

	class FloatingObject {
	public:
		DWORD lastDeterminationTime;
		const size_t velocityBufferSize;
		std::deque<Eigen::Vector3f> positionBuffer;
		std::deque<Eigen::Vector3f> velocityBuffer;
		std::deque<float> dTBuffer;
		float m_radius; // [mm]
		float m_weight;

	private:
		Eigen::Vector3f m_position;
		Eigen::Vector3f m_velocity;
		Eigen::Vector3f m_integral;
		Eigen::Vector3f m_upperbound;
		Eigen::Vector3f m_lowerbound;
		bool m_is_tracked;
		std::mutex mtxState;
		std::mutex mtxTrack;
		std::shared_mutex mtxTrajectory;
		std::shared_ptr<Trajectory> trajectoryPtr;

	public:
		FloatingObject(
			const Eigen::Vector3f& positionTarget,
			const Eigen::Vector3f& lowerbound,
			const Eigen::Vector3f& upperbound,
			float radius,
			float weight = 0.0f
		);

		static FloatingObjectPtr Create(
			const Eigen::Vector3f& posTgt,
			const Eigen::Vector3f& lowerbound,
			const Eigen::Vector3f& upperbound,
			float radius,
			float weight = 0.0f
		);

		float sphereMass(); //return a mass equivalent to an air of the volume of the sphere
		float weight();
		float radius();
		float totalMass();
		Eigen::Vector3f getPosition();
		Eigen::Vector3f getVelocity();
		Eigen::Vector3f getIntegral();
		Eigen::Vector3f getPositionTarget(DWORD systime_ms = timeGetTime());
		Eigen::Vector3f getVelocityTarget(DWORD systime_ms = timeGetTime());
		Eigen::Vector3f getAccelTarget(DWORD systime_ms = timeGetTime());
		void getStates(Eigen::Vector3f& pos, Eigen::Vector3f& vel, Eigen::Vector3f& integ);
		void updateStates(DWORD determinationTime, const Eigen::Vector3f& positionNew);
		void resetIntegral();

		void setTrajectory(std::shared_ptr<Trajectory> newTrajectoryPtr);
		void updateStatesTarget(
			const Eigen::Vector3f& positionTarget,
			const Eigen::Vector3f& velocityTarget = Eigen::Vector3f::Zero(),
			const Eigen::Vector3f& accelTarget = Eigen::Vector3f::Zero()
		);

		bool isConverged(float tolPos, float tolVel);
		bool isTracked();
		bool isInsideWorkspace();
		void setTrackingStatus(bool is_tracked);
		Eigen::Vector3f averageVelocity(
			std::deque<Eigen::Vector3f> velBuffer,
			std::deque<float> intervalBuffer
		);
		Eigen::Vector3f averagePosition();
		Eigen::Vector3f lowerbound();
		Eigen::Vector3f upperbound();
	};
}
