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
	
	ocs c;
	c.AddDevice(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0));
	c.AddDevice(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, M_PI, 0));

	std::cout << c.DirectionsAUTD() << std::endl;;
	return 0;

	//std::ofstream ofs("20190307_single_sided2.csv");
	//ofs << "t, x, y, z, focus_x, focus_y, focus_z " << std::endl;
	const float xDevice = 192.f;
	const float yDevice = 151.4f;
	const int rowDevice = 3;
	const int colDevice = 3;
	
	ods sensor;
	sensor.Initialize();
	sensor.SetWorkSpace(Eigen::Vector3f(xDevice, 0, 445.f), Eigen::Vector3f(xDevice * colDevice, yDevice * rowDevice, 1000.f));
	sensor.SetSensorGeometry(Eigen::Vector3f(xDevice * colDevice / 2.f, -472.f, 445.f), Eigen::Vector3f(M_PI_2, M_PI_2, M_PI_2));
	cv::Mat mask;
	sensor.MaskWorkspace(mask);

	while (1) {
		sensor.GetPositionByDepth(FloatingObject::Create(Eigen::Vector3f(0, 0, 1500)), Eigen::Vector3f(), false);
		auto key = cv::waitKey(1);
		if (key == '27') { break; }
	}
	return 0;

	autd::Controller autd;
	autd.Open(autd::LinkType::ETHERCAT);
	if (!autd.isOpen()) return ENXIO;

	const int nDevice = rowDevice * colDevice;
	for (int i = 0; i < nDevice; i++) {
		Eigen::Vector3f posAUTD((i % colDevice) * xDevice, (i / colDevice) * yDevice, 0);
		autd.geometry()->AddDevice(posAUTD, Eigen::Vector3f::Zero());
		std::cout << i << ": " << posAUTD.transpose() << std::endl;
	}

	//control parameters
	Eigen::Vector3f gainP = Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = Eigen::Vector3f::Constant(-0.05f);
	float z0 = 150.f, F0 = 3.9f;
	//SetSensor
	FloatingObjectPtr objPtr = FloatingObject::Create(Eigen::Vector3f(273, 151.2 * 1.5, 300), 0.1);
	while (1) {
		system_clock::time_point beginLoop = system_clock::now();
		Eigen::Vector3f position;
		DWORD observationTime = timeGetTime();
		bool succeeded = sensor.GetPositionByDepth(objPtr, position, false);
		if (succeeded) {
			objPtr->updateStates(observationTime, position);
			Eigen::Vector3f dr = objPtr->getPosition() - objPtr->getPositionTarget();
			Eigen::Vector3f accel
				= gainP.asDiagonal() * (objPtr->getPosition() - objPtr->getPositionTarget())
				+ gainD.asDiagonal() * (objPtr->getVelocity() - objPtr->getVelocityTarget())
				+ gainI.asDiagonal() * objPtr->getIntegral()
				+ objPtr->getAccelTarget();
			Eigen::Vector3f force = objPtr->totalMass() * accel + objPtr->AdditionalMass()*Eigen::Vector3f(0.f, 0.f, 9.80665f);
			Eigen::Vector3f focus = objPtr->getPosition() - objPtr->Radius() * force.normalized();
			Eigen::Vector3f shift = focus - position;
			float cosPhi = force.normalized().dot(Eigen::Vector3f::UnitZ());
			float denom = (objPtr->getPosition().z() - objPtr->Radius() * cosPhi);
			float duty = force.norm() / F0 * z0 * z0 / cosPhi / denom / denom;
			int amplitude = static_cast<int>(510.f / M_PI * asin(sqrt(std::min(std::max(0.f, duty), 1.f))));
			autd.AppendGainSync(autd::GaussianBeamGain::Create(focus, force, 255, M_PI / 6));
			//std::cout << "DETECTED: " << position.transpose() << ", amplitude: " << amplitude << std::endl;
			//ofs << observationTime << ", " << dr.x() << ", " << dr.y() << ", " << dr.z() << ", " << shift.x() << ", " << shift.y() << ", " << shift.z() << std::endl;
		}
		while (duration_cast<milliseconds>(system_clock::now() - beginLoop).count() < 33) {
			//busy_wait
		}

		//Determine
		//if(isTracked) AppendGain
	}

}