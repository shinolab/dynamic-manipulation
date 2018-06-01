#ifndef _ODCS_H_
#define _ODCS_H_

#include "autd3.hpp"
#include "KinectApp.hpp"
//#include "engine.h"
#include <queue>
#include <vector>
#include <Eigen\Dense>
#include <Windows.h>

#pragma comment (lib, "winmm")

class FloatingObject{
public:
	Eigen::Vector3f position;
	Eigen::Vector3f velocity;
	Eigen::Vector3f integral;
	Eigen::Vector3f positionTarget;
	Eigen::Vector3f velocityTarget;
	Eigen::Vector3f force_offset;
	DWORD lastDeterminationTime;
	bool isTracked;
	bool _isStable;
	bool isControlled;
	std::deque<Eigen::Vector3f> velocityBuffer;
	std::deque<float> dTBuffer;
	const int velocityBufferSize = 3;
	const float radius = 70.0; // [mm]
	const float mass = 5.4e-3; //[Kg]
	const float speedLimit = 400; // [mm/s]
	Eigen::Vector3f gravityForce;
	Eigen::MatrixXf statePath;
	Eigen::MatrixXf derivativePath;
	Eigen::MatrixXf controlPath;
	Eigen::RowVectorXf timePath;

	FloatingObject(Eigen::Vector3f _positionTarget);

	void updateStates(DWORD determinationTime, Eigen::Vector3f positionNew);

	void updateStatesTarget(Eigen::Vector3f _positionTarget, Eigen::Vector3f _velocityTarget);

	bool isStable();

	Eigen::Vector3f averageVelocity();
};

class ods
{
public:
	KinectApp kinectApp;
	Eigen::Vector3f positionKinect = Eigen::Vector3f::Zero();
	Eigen::Matrix3f dcmGlobal2Kinect = Eigen::Matrix3f::Identity(3, 3);
	Eigen::Matrix3f dcmKinect2Global = Eigen::Matrix3f::Identity(3, 3); // x_kinect = attitudeKinect * x_global
	Eigen::Affine3f affineKinect2Global;
	Eigen::Matrix<float, 3, 2> workspace;

public:
	int Initialize();

	bool isInsideWorkSpace(Eigen::Vector3f pos);

	void DeterminePositionByHSV(FloatingObject* obj, cv::Scalar lb, cv::Scalar ub);

	void DeterminePositionByBGR(FloatingObject* obj, cv::Scalar lb, cv::Scalar ub);

	void DeterminePositionByDepth(FloatingObject* obj);

	void DeterminePositionByDepth(std::vector<FloatingObject> &objs);

	void GetMarkerPosition();

};

class ocs
{
public:
	autd::Controller autd;
	Eigen::MatrixXf positionAUTD;
	Eigen::MatrixXf directionsAUTD;
	Eigen::MatrixXf eulerAnglesAUTD;
	Eigen::MatrixXf centerAUTD;

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

	Eigen::VectorXf FindDutyQP(FloatingObject* obj);

	//Find Duty Module (with offset)

	void FindDutyBruteForce(Eigen::VectorXf *const duties, Eigen::MatrixXf *const directions, Eigen::Vector3f pos, Eigen::Vector3f force);

	int findDutyNLP(Eigen::VectorXf *duties, Eigen::MatrixXf *farpoints, Eigen::Vector3f pos, Eigen::Vector3f force);

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

	void PIDControl(FloatingObject* obj);

	void DirectSemiPlaneWave(FloatingObject* obj, Eigen::VectorXi amplitudes);

	//Global Control Module

	void PositionControlBySingleAUTD(FloatingObject* obj);

	int SetPath(Eigen::Vector3f initialPosition, Eigen::Vector3f finalPosition, FloatingObject* obj);

	void followPath(FloatingObject* obj);

	//Legacy Module
	Eigen::VectorXf findDutySI(FloatingObject* obj);

	Eigen::VectorXf findDutyQPEq(FloatingObject* obj);
};

#endif
