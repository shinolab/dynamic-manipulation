#include "odcs.hpp"
#include "additionalGain.hpp"
#include <iostream>

int main() {

	ods sensor;
	sensor.Initialize();
	sensor.SetWorkSpace(Eigen::Vector3f(-1000, 0, 305.f), Eigen::Vector3f(1000.f, 1100.f, 2000.f));
	sensor.SetSensorGeometry(Eigen::Vector3f(105.f, -1031.f, 573.f), Eigen::Vector3f(M_PI_2, M_PI_2, M_PI_2));

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

	Eigen::Vector3f posTgt = controller.CentersAUTD().col(3) + Eigen::Vector3f(0, 0, -1000);//just below AUTD #4
	FloatingObjectPtr objPtr = FloatingObject::Create(posTgt, -0.1e-3f);
	//Control Loop
	DWORD observationTime = timeGetTime();
	//Eigen::Vector3f posObserved = posTgt + Eigen::Vector3f(0, 0, 100.f);
	//bool succeeded = sensor.GetPositionByDepth(objPtr, posObserved, true);
	for(int i = 0; i < 800; i+=2)
	{
		Eigen::Vector3f posObserved = posTgt + Eigen::Vector3f(0, 0, -400 + i);

		//----------Determination----------
		objPtr->updateStates(observationTime, posObserved);
		objPtr->SetTrackingStatus(true);
		Eigen::Vector3f accel
			= gainP.asDiagonal() * (posObserved - posTgt)
			//+ gainD.asDiagonal() * (objPtr->getVelocity() - objPtr->getVelocityTarget())
			//+ gainI.asDiagonal() * objPtr->getIntegral()
			+ objPtr->getAccelTarget();
		//======OK======
		Eigen::Vector3f forceToApply = objPtr->totalMass() * accel + objPtr->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665e3f);
		Eigen::VectorXf duties = controller.FindDutyQP(forceToApply, objPtr->getPosition());
		Eigen::VectorXi amplitudes = (510.f / M_PI * duties.array().max(0.f).min(1.f).sqrt().asin().matrix()).cast<int>();
		Eigen::MatrixXf focus = objPtr->getPosition().replicate(1, controller.CentersAUTD().cols());
		//controller._autd.AppendGainSync(autd::DeviceSpecificFocalPointGain::Create(focus, amplitudes));
		Eigen::Vector3f force_result = controller.arfModelPtr->arf(posObserved.replicate(1, controller._autd.geometry()->numDevices()) - controller.CentersAUTD(), controller.eulerAnglesAUTD) * duties;
		Eigen::Vector3f posTgt = objPtr->getPositionTarget();
		std::cout << observationTime << ", " << posObserved.x() << ", " << posObserved.y() << ", " << posObserved.z() << ", "
			<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", "
			<< accel.x() << ", " << accel.y() << ", " << accel.z() << ", "
			<< forceToApply.x() << ", " << forceToApply.y() << ", " << forceToApply.z() << ","
			<< force_result.x() << ", " << force_result.y() << ", " << force_result.z() << ", ";
		
		for (int i = 0; i < controller._autd.geometry()->numDevices(); i++) {
			std::cout << amplitudes[i] << ", ";
		}
		std::cout << std::endl;
	}
	getchar();
	return 0;
}