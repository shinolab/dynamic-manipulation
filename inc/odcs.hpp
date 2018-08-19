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

class FloatingObject{
public:
	Eigen::Vector3f getPosition();
	Eigen::Vector3f getVelocity();
	Eigen::Vector3f getIntegral();
	Eigen::Vector3f getPositionTarget();
	Eigen::Vector3f getVelocityTarget();
	Eigen::Matrix3f covError;
	
	DWORD lastDeterminationTime;
	bool isTracked;
	bool _isStable;
	bool isControlled;
	std::deque<Eigen::Vector3f> velocityBuffer;
	std::deque<float> dTBuffer;
	const int velocityBufferSize = 3;
	float radius = 100.0; // [mm]
	//const float mass = 5.4e-3; //[Kg]
	const float speedLimit = 400; // [mm/s]
	Eigen::Vector3f gravityForce;
	Eigen::MatrixXf statePath;
	Eigen::MatrixXf derivativePath;
	Eigen::MatrixXf controlPath;
	Eigen::RowVectorXf timePath;
	float additionalMass = 0.1e-3;


private:
	Eigen::Vector3f position;
	Eigen::Vector3f velocity;
	Eigen::Vector3f integral;
	Eigen::Vector3f positionTarget;
	Eigen::Vector3f velocityTarget;
	Eigen::Vector3f force_offset;
	std::mutex mtxState;
	std::mutex mtxStateTarget;


public:
	FloatingObject(Eigen::Vector3f _positionTarget);

	float sphereMass(); //return a mass equivalent to an air of the volume of the sphere

	float totalMass(); 

	void updateStates(DWORD determinationTime, Eigen::Vector3f positionNew);

	void updateStates(DWORD determinationTime, Eigen::Vector3f positionNew, Eigen::Vector3f velocitynew);

	void updateStatesTarget(Eigen::Vector3f _positionTarget, Eigen::Vector3f _velocityTarget);

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

	bool isInsideWorkSpace(Eigen::Vector3f pos);

	Eigen::Vector3f getPositionAtCGI(cv::Mat depthImage, float radius, bool isBinary);

	void DeterminePositionByHSV(FloatingObjectPtr objPtr, cv::Scalar lb, cv::Scalar ub);

	void DeterminePositionByBGR(FloatingObjectPtr objPtr, cv::Scalar lb, cv::Scalar ub);

	void DeterminePositionByDepth(FloatingObjectPtr objPtr);

	void DeterminePositionByDepthWithROI(FloatingObjectPtr objPtr);

	void DeterminePositionByDepth(std::vector<FloatingObjectPtr> objPtrs);

	//This function only observes a position of the object and do NOT update its position.
	bool GetPositionByDepthWithROI(FloatingObjectPtr objPtr, Eigen::Vector3f &pos);

	void GetMarkerPosition();

	Eigen::Affine3f getAffineKinect2Global() { return affineKinect2Global; }

};

class ocs
{
public:
	autd::Controller autd;
	Eigen::MatrixXf positionAUTD;
	Eigen::MatrixXf directionsAUTD;
	Eigen::MatrixXf eulerAnglesAUTD;
	Eigen::MatrixXf centerAUTD;
	std::unique_ptr<arfModelTheoreticalTable> arfModelPtr;
private:
	Eigen::Vector3f gainP = -0.1 * Eigen::Vector3f::Ones();
	Eigen::Vector3f gainD = -0.2 * Eigen::Vector3f::Ones();
	Eigen::Vector3f gainI = Eigen::Vector3f::Zero();
	
	//Engine *ep;

public:
	int Initialize();

	void Close();

	//Find Duty Module (without offset)

	Eigen::VectorXf FindDutyQP(Eigen::Vector3f force, Eigen::Vector3f position);

	Eigen::VectorXf FindDutyQP(FloatingObjectPtr objPtr);

	Eigen::VectorXf FindDutySVD(FloatingObjectPtr objPtr);

	//Find Duty Module (with offset)

	void FindDutyBruteForce(Eigen::VectorXf *const duties, Eigen::MatrixXf *const directions, Eigen::Vector3f pos, Eigen::Vector3f force);

	//Objective Functions

	static double SqDiffOfForce(const std::vector<double> &x, std::vector<double> &grad, void* data);

	struct dataObj{
	public:
		Eigen::MatrixXf posRel;
		Eigen::Vector3f forceTarget;
		Eigen::MatrixXf posAUTDS;
		Eigen::MatrixXf eulerAngleAUTDS;
	};

	//Local Control Module

	void DirectSemiPlaneWave(FloatingObjectPtr objPtr, Eigen::VectorXi amplitudes);

	//Global Control Module

	void PositionControlBySingleAUTD(FloatingObjectPtr objPtr);

	void followPath(FloatingObjectPtr objPtr);

	//Legacy Module
	Eigen::VectorXf FindDutySI(FloatingObjectPtr objPtr);

	Eigen::VectorXf findDutyQPEq(FloatingObjectPtr objPtr);
};

class odcs
{
public:
	void Initialize();
	void StartControl(std::vector<FloatingObjectPtr> &objPtrs);
	void ControlLoop(std::vector<FloatingObjectPtr> objPtrs);
	void Close();
	void DetermineStateKF(FloatingObjectPtr objPtr, Eigen::VectorXf amplitudes);
	ods ods;
	ocs ocs;
	std::thread thread_control;
private:
};

#endif
