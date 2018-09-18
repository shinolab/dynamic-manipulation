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
#include <Eigen/Dense>
#define NOMINMAX
#include <Windows.h>

#pragma comment (lib, "winmm")

class FloatingObject {
public:
	Eigen::Vector3f getPosition();
	Eigen::Vector3f getVelocity();
	Eigen::Vector3f getIntegral();
	Eigen::Vector3f getPositionTarget();
	Eigen::Vector3f getVelocityTarget();
	Eigen::VectorXf getLatestInput();
	void setLatestInput(Eigen::VectorXf input);
	Eigen::VectorXf inputLatest;

	Eigen::MatrixXf covError;
	DWORD lastDeterminationTime;
	bool isTracked;
	bool _isStable;
	bool isControlled;
	std::deque<Eigen::Vector3f> velocityBuffer;
	std::deque<float> dTBuffer;
	const int velocityBufferSize = 3;
	float radius = 100.0; // [mm]
	//const float mass = 5.4e-3; //[Kg]
	float speedLimit = 400; // [mm/s]
	float additionalMass = 0.1e-3;

private:
	Eigen::Vector3f position;
	Eigen::Vector3f velocity;
	Eigen::Vector3f integral;
	Eigen::Vector3f positionTarget;
	Eigen::Vector3f velocityTarget;
	std::mutex mtxState;
	std::mutex mtxStateTarget;

public:
	FloatingObject(Eigen::Vector3f _positionTarget);

	float sphereMass(); //return a mass equivalent to an air of the volume of the sphere

	float totalMass(); 

	void updateStates(DWORD determinationTime, Eigen::Vector3f &positionNew);

	void updateStates(DWORD determinationTime, Eigen::Vector3f &positionNew, Eigen::Vector3f &velocitynew);

	void updateStatesTarget(Eigen::Vector3f &_positionTarget, Eigen::Vector3f &_velocityTarget);

	bool isStable();

	Eigen::Vector3f averageVelocity();
};

typedef std::shared_ptr<FloatingObject> FloatingObjectPtr;

class ods
{
private:
	KinectApp kinectApp;
	Eigen::Vector3f positionKinect;
	Eigen::Matrix3f dcmGlobal2Kinect;
	Eigen::Matrix3f dcmKinect2Global; // x_kinect = attitudeKinect * x_global
	Eigen::Affine3f affineKinect2Global;
	Eigen::Matrix<float, 3, 2> workspace;

public:
	int Initialize();

	Eigen::Affine3f getAffineKinect2Global() { return affineKinect2Global; }

	Eigen::Matrix3f getDcmGlobal2Kinect() { return dcmGlobal2Kinect; }

	bool isInsideWorkSpace(const Eigen::Vector3f &pos);

	void DeterminePositionByHSV(FloatingObjectPtr objPtr, cv::Scalar lb, cv::Scalar ub);

	void DeterminePositionByBGR(FloatingObjectPtr objPtr, cv::Scalar lb, cv::Scalar ub);

	void DeterminePositionByDepth(FloatingObjectPtr objPtr, bool useROI);

	void DeterminePositionByDepth(std::vector<FloatingObjectPtr> objPtrs);

	bool GetPositionByBGR(FloatingObjectPtr objPtr, Eigen::Vector3f &pos, cv::Scalar lb, cv::Scalar ub);

	bool GetPositionByHSV(FloatingObjectPtr objPtr, Eigen::Vector3f &pos, cv::Scalar lb, cv::Scalar ub);

	//This function only observes a position of the object and do NOT update its position.
	bool GetPositionByDepth(FloatingObjectPtr objPtr, Eigen::Vector3f &pos, bool useROI);

};

class ocs
{
public:
	autd::Controller autd;
	Eigen::MatrixXf positionAUTD;
	Eigen::MatrixXf directionsAUTD;
	Eigen::MatrixXf eulerAnglesAUTD;
	Eigen::MatrixXf centerAUTD;
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

	void SetArfModel(std::unique_ptr<arfModelLinearBase> arfModelPtr);

	void SetGain(Eigen::Vector3f gainP, Eigen::Vector3f gainD, Eigen::Vector3f gainI);

	Eigen::Vector3f ComputePIDForce(FloatingObjectPtr objPtr);

	Eigen::VectorXf FindDutyQP(Eigen::Vector3f force, Eigen::Vector3f position);

	Eigen::VectorXf FindDutyQP(FloatingObjectPtr objPtr);

	Eigen::VectorXf FindDutySVD(FloatingObjectPtr objPtr);
	
	void DirectSemiPlaneWave(FloatingObjectPtr objPtr, Eigen::VectorXi amplitudes);
	
	//Legacy Module
	Eigen::VectorXf FindDutySI(FloatingObjectPtr objPtr);

	Eigen::VectorXf FindDutyQPEq(FloatingObjectPtr objPtr);
};

class odcs
{
public:
	void Initialize();
	int AddObject(Eigen::Vector3f positionTarget);
	void RegisterObject(FloatingObjectPtr objPtr);
	const FloatingObjectPtr GetFloatingObject(int i);
	void StartControl();
	void ControlLoop(std::vector<FloatingObjectPtr> &objPtrs, int loopPeriod);
	void Close();
	void DetermineStateKF(FloatingObjectPtr objPtr, const Eigen::Vector3f observe, const DWORD determinationTime);
	ods ods;
	ocs ocs;
	std::thread thread_control;
private:
	std::vector<FloatingObjectPtr> objPtrs;
};

#endif
