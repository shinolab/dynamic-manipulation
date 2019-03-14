#include "odcs.hpp"
#include "additionalGain.hpp"
#include "autd3.hpp"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std::chrono;

int main() {
	
	
	ods sensor;
	sensor.Initialize();
	sensor.SetWorkSpace(Eigen::Vector3f(-1000, 0, 245.f), Eigen::Vector3f(1000.f, 1000.f, 2000.f));
	sensor.SetSensorGeometry(Eigen::Vector3f(105.f, -1031.f, 573.f), Eigen::Vector3f(M_PI_2, M_PI_2, M_PI_2));
	cv::Mat mask;
	sensor.MaskWorkspace(mask);

	ocs controller;
	controller.Initialize();
	controller.AddDevice(Eigen::Vector3f(-520.f, 450.f, 0.f), Eigen::Vector3f::Zero());
	controller.AddDevice(Eigen::Vector3f(-260.f, 225.f, 0.f), Eigen::Vector3f::Zero());
	controller.AddDevice(Eigen::Vector3f(-260.f, 675.f, 0.f), Eigen::Vector3f::Zero());
	controller.AddDevice(Eigen::Vector3f(0.f, 0.f, 0.f), Eigen::Vector3f::Zero());
	controller.AddDevice(Eigen::Vector3f(0.f, 450.f, 0.f), Eigen::Vector3f::Zero());
	controller.AddDevice(Eigen::Vector3f(0.f, 900.f, 0.f), Eigen::Vector3f::Zero());
	controller.AddDevice(Eigen::Vector3f(260.f, 225.f, 0.f), Eigen::Vector3f::Zero());
	controller.AddDevice(Eigen::Vector3f(260.f, 675.f, 0.f), Eigen::Vector3f::Zero());
	controller.AddDevice(Eigen::Vector3f(520.f, 450.f, 0.f), Eigen::Vector3f::Zero());
	//control parameters
	Eigen::Vector3f gainP = Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = Eigen::Vector3f::Constant(-0.05f);
	controller.SetGain(gainP, gainD, gainI);

	FloatingObjectPtr objPtr = FloatingObject::Create(Eigen::Vector3f(0.f, 450.f, 1500));
	
	while (1) {
		DWORD observationTime = timeGetTime();
		Eigen::Vector3f posObserved;
		bool succeeded = sensor.GetPositionByDepth(objPtr, posObserved, false);
		if (succeeded && sensor.isInsideWorkSpace(posObserved))
		{
			//----------Determination----------
			objPtr->updateStates(observationTime, posObserved);
			objPtr->SetTrackingStatus(true);
			Eigen::Vector3f accel
				= gainP.asDiagonal() * (objPtr->getPosition() - objPtr->getPositionTarget())
				+ gainD.asDiagonal() * (objPtr->getVelocity() - objPtr->getVelocityTarget())
				+ gainI.asDiagonal() * objPtr->getIntegral()
				+ objPtr->getAccelTarget();
			Eigen::Vector3f forceToApply = objPtr->totalMass() * accel + objPtr->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665f);
			Eigen::VectorXf duties = controller.FindDutyQP(forceToApply, objPtr->getPosition());
			Eigen::VectorXi amplitudes = (510.f / M_PI * duties.array().max(0.f).min(1.f).sqrt().asin().matrix()).cast<int>();
			Eigen::MatrixXf focus = controller.CentersAUTD() + (objPtr->getPosition().replicate(1, controller.CentersAUTD().cols()) - controller.CentersAUTD());
			controller._autd.AppendGainSync(autd::DeviceSpecificFocalPointGain::Create(focus, amplitudes));
		}
		else if (observationTime - objPtr->lastDeterminationTime > 1000)
		{
			objPtr->SetTrackingStatus(false);
		}
		
		while (timeGetTime() - observationTime < 33) {
			//busy_wait
		}

		//Determine
		//if(isTracked) AppendGain
	}

}