#ifndef _DYNAMAN_FLOATINGOBJECT_HPP
#define _DYNAMAN_FLOATINGOBJECT_HPP

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
}

#endif