#include "odcs.hpp"
#include "additionalGain.hpp"
#include "autd3.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std::chrono;

int main() {
	
	autd::Controller autd;
	autd.Open(autd::LinkType::ETHERCAT);
	if (!autd.isOpen()) return ENXIO;
	
	const int nDevice = 12;
	for (int i = 0; i < nDevice; i++){
		Eigen::Vector3f posAUTD((i % 2) * 200, (i / 2) * 150, 0);
		autd.geometry()->AddDevice(posAUTD, Eigen::Vector3f::Zero());
	}

	//control parameters
	Eigen::Vector3f gainP = Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = Eigen::Vector3f::Constant(-0.05f);
	float z0 = 200.f, F0 = 0.1f;
	ods sensor;
	sensor.Initialize();
	sensor.SetSensorGeometry(Eigen::Vector3f(0, 0, 200), Eigen::Vector3f(M_PI_2, -M_PI_2, -M_PI_2));
	//SetSensor
	FloatingObjectPtr objPtr = FloatingObject::Create(Eigen::Vector3f(0, 0, 400));
	while (1) {
		system_clock::time_point beginLoop = system_clock::now();
		Eigen::Vector3f position;
		HRESULT hr = sensor.GetPositionByDepth(objPtr, position, true);
		if (SUCCEEDED(hr)) {
			objPtr->updateStates(timeGetTime(), position);
			Eigen::Vector3f accel
				= gainP.asDiagonal() * (objPtr->getPosition() - objPtr->getPositionTarget())
				+ gainD.asDiagonal() * (objPtr->getVelocity() - objPtr->getVelocityTarget())
				+ gainI.asDiagonal() * objPtr->getIntegral()
				+ objPtr->getAccelTarget();
			Eigen::Vector3f force = objPtr->totalMass() * accel + objPtr->AdditionalMass()*Eigen::Vector3f(0.f, 0.f, 9.80665f);
			Eigen::Vector3f focus = objPtr->getPosition() - objPtr->Radius() * force.normalized();
			float cosPhi = force.normalized().dot(Eigen::Vector3f::UnitZ());
			float denom = (objPtr->getPosition().z() - objPtr->Radius() * cosPhi);
			float duty = force.norm() / F0 * z0 * z0 / cosPhi / denom / denom;
			int amplitude = static_cast<int>(510.f / M_PI * asin(sqrt(std::min(std::max(0.f, duty), 1.f))));
			autd.AppendGainSync(autd::GaussianBeamGain::Create(focus, Eigen::Vector3f::UnitZ(), amplitude, M_PI / 3));
			while(duration_cast<milliseconds>(system_clock::now() - beginLoop).count() < 33){
				//busy_wait
			}
		}
		//Determine
		//if(isTracked) AppendGain
	}

}