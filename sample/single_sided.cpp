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
	
	std::ofstream ofs("20190323_ceiling_sided_PID_stationary3.csv");
	ofs << "t, x, y, z, x_tgt, y_tgt, z_tgt, Fx, Fy, Fz, Fx_tgt, Fy_tgt, Fz_tgt,";
	for (int i = 0; i < 12; i++) {
		ofs << "amp" << i << ",";
	}
	ofs << std::endl;
	ods sensor;
	sensor.Initialize();
	sensor.SetWorkSpace(Eigen::Vector3f(-800.f, 0.f, 500.f), Eigen::Vector3f(800.f, 1000.f, 1570.f));;
	//sensor.SetSensorGeometry(Eigen::Vector3f(40.f, -1241.f, 1080.f), Eigen::Vector3f(M_PI_2, M_PI_2, M_PI_2));
	Eigen::Matrix3f rotationKinect2Global;
	rotationKinect2Global <<
		-0.999986, -0.000739038, 0.00521048,
		0.00518096, 0.0355078, 0.999356,
		-0.000923425, 0.999369, -0.0355035;

	sensor.SetSensorGeometry(Eigen::Vector3f(35.1867f, -1242.32f, 1085.62f), rotationKinect2Global);
	cv::Mat mask;
	sensor.MaskWorkspace(mask);

	FloatingObjectPtr objPtr = FloatingObject::Create(Eigen::Vector3f(-300, 530+65, 931), -0.1e-4f);

	ocs controller;
	controller.Initialize();
	controller.AddDevice(Eigen::Vector3f(992.5f, 270.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));	
	controller.AddDevice(Eigen::Vector3f(992.5f, 790.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	controller.AddDevice(Eigen::Vector3f(542.5f, 10.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	controller.AddDevice(Eigen::Vector3f(542.5f, 530.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	controller.AddDevice(Eigen::Vector3f(542.5f, 1050.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	controller.AddDevice(Eigen::Vector3f(92.5f, 270.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	controller.AddDevice(Eigen::Vector3f(92.5f, 790.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	controller.AddDevice(Eigen::Vector3f(-357.5f, 10.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	controller.AddDevice(Eigen::Vector3f(-357.5f, 530.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	controller.AddDevice(Eigen::Vector3f(-357.5f, 1050.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	controller.AddDevice(Eigen::Vector3f(-807.5f, 270.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));
	controller.AddDevice(Eigen::Vector3f(-807.5f, 790.f, 1931.f), Eigen::Vector3f(0.f, M_PI, 0.f));


	//control parameters
	Eigen::Vector3f gainP = Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = Eigen::Vector3f::Constant(-0.05f);
	controller.SetGain(gainP, gainD, gainI);

	while (1) {
		DWORD observationTime = timeGetTime();
		Eigen::Vector3f posObserved;
		bool succeeded = sensor.GetPositionByDepth(objPtr, posObserved, true);
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
			Eigen::Vector3f forceToApply = objPtr->totalMass() * accel + objPtr->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665e3f);
			Eigen::VectorXf duties = controller.FindDutySelectiveQP(forceToApply, objPtr->getPosition());
			Eigen::VectorXi amplitudes = (510.f / M_PI * duties.array().max(0.f).min(1.f).sqrt().asin().matrix()).cast<int>();
			Eigen::MatrixXf focus =  objPtr->getPosition().replicate(1, controller.CentersAUTD().cols());
			controller._autd.AppendGainSync(autd::DeviceSpecificFocalPointGain::Create(focus, amplitudes));
			Eigen::Vector3f force_result = controller.arfModelPtr->arf(posObserved.replicate(1, controller._autd.geometry()->numDevices()) - controller.CentersAUTD(), controller.eulerAnglesAUTD) * duties;
			Eigen::Vector3f posTgt = objPtr->getPositionTarget();
			ofs << observationTime << ", " << posObserved.x() << ", " << posObserved.y() << ", " << posObserved.z() << ", "
				<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", "
				<< force_result.x() << ", " << force_result.y() << ", " << force_result.z() << ", "
				<< forceToApply.x() << ", " << forceToApply.y() << ", " << forceToApply.z() << ",";
			for (int i = 0; i < controller._autd.geometry()->numDevices(); i++) {
				ofs << amplitudes[i] << ", ";
				}
				ofs<< std::endl;
		}
		else if (observationTime - objPtr->lastDeterminationTime > 1000)
		{
			objPtr->SetTrackingStatus(false);
		}
		
		while (timeGetTime() - observationTime < 33) {
			Sleep(0);//busy_wait
		}

	}

}