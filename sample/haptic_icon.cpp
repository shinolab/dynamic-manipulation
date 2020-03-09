#include "odcs.hpp"
#include "additionalGain.hpp"
#include "autd3.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <Windows.h>

#pragma comment(lib, "winmm")

const float pi = 3.14159265359f;
const int num_trans_in_device = 249;

namespace haptic_icon {
	int CustomAddDevice(autd::GeometryPtr geometry, const float theta, const float phi, bool isUpper) {
		const float radius = 528.f;
		const float dia_trans = 10.16f;
		Eigen::Vector3f centerLocal(dia_trans * 8.5f, dia_trans * 6.5f, 0.f);

		Eigen::Vector3f euler_angles;
		isUpper ? euler_angles << phi, theta - pi, pi / 2 : euler_angles << phi, theta - pi, -pi / 2;
		Eigen::Vector3f centerGlobal = radius* Eigen::Vector3f(sinf(theta) * cosf(phi), sinf(theta) * sinf(phi), cosf(theta));
		Eigen::Vector3f pos_autd = centerGlobal
			- Eigen::AngleAxisf(euler_angles.x(), Eigen::Vector3f::UnitZ())
			* Eigen::AngleAxisf(euler_angles.y(), Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(euler_angles.z(), Eigen::Vector3f::UnitZ()) * centerLocal;

		std::cout << "AUTD # " << geometry->numDevices() << std::endl
			<< "position: " << pos_autd.transpose() << std::endl
			<< "euler_angles(ZYZ): " << euler_angles.transpose() / pi *180<< std::endl;
		return geometry->AddDevice(pos_autd, euler_angles);
	}
}

int main(int argc, char** argv) {
	autd::Controller autd;
	autd.Open(autd::LinkType::ETHERCAT);
	if (!autd.isOpen()) {
		return ENXIO;
	}
	//top autd
	const float theta0 = 0.f, phi0 = -3 * pi / 4;
	//upper autd
	const float theta1 = pi / 4, phi1 = pi / 4;
	const float theta2 = pi / 4, phi2 = 3 * pi / 4;
	const float theta3 = pi / 4, phi3 = - 3 * pi / 4;
	const float theta4 = pi / 4, phi4 = - pi / 4;
	//middle autd
	const float theta5 = pi / 2, phi5 = pi / 2;
	const float theta6 = pi / 2, phi6 = 3 * pi / 2;
	//lower autd
	const float theta7 = 3 * pi / 4, phi7 = pi / 4;
	const float theta8 = 3 * pi / 4, phi8 = 3 * pi / 4;
	const float theta9 = 3 * pi / 4, phi9 = -3 * pi / 4;
	const float theta10 = 3 * pi / 4, phi10 = - pi / 4;

	haptic_icon::CustomAddDevice(autd.geometry(), theta9, phi9, false);
	autd.geometry()->AddDevice(Eigen::Vector3f(-528.f, 10.16f * 6.5f, -10.16f * 8.5f), Eigen::Vector3f(0, pi / 2, pi));// AUTD #6 (Middle-left)
	haptic_icon::CustomAddDevice(autd.geometry(), theta8, phi8, false);
	haptic_icon::CustomAddDevice(autd.geometry(), theta7, phi7, false);
	autd.geometry()->AddDevice(Eigen::Vector3f(528.f, -10.16f * 6.5f, -10.16f * 8.5f), Eigen::Vector3f(0, -pi / 2, 0));// AUTD #5 (Middle-right)
	haptic_icon::CustomAddDevice(autd.geometry(), theta10, phi10, false);
	haptic_icon::CustomAddDevice(autd.geometry(), theta4, phi4, true);
	haptic_icon::CustomAddDevice(autd.geometry(), theta1, phi1, true);
	haptic_icon::CustomAddDevice(autd.geometry(), theta2, phi2, true);
	haptic_icon::CustomAddDevice(autd.geometry(), theta3, phi3, true);
	haptic_icon::CustomAddDevice(autd.geometry(), theta0, phi0, true);
	Eigen::MatrixXf foci(3, autd.geometry()->numDevices());
	Eigen::Vector3f point(0, 0, 0);
	autd.AppendGainSync(autd::FocalPointGain::Create(point));
	autd.AppendModulationSync(autd::Modulation::Create(255));

	
	//auto initTime = timeGetTime();
	//int m = rand();
	//int sign;
	//m % 2 == 0 ? sign = 1 : sign = -1;
	//while (timeGetTime() - initTime < 30000) {
	//	//float x = -150 + 300.f * float((timeGetTime() - initTime) % 5000) / 5000.f;
	//	float z = -150 + 300.f * float((timeGetTime() - initTime) % 3000) / 3000.f;

	//	point << 0, 0, sign*z;
	//	autd.AppendGainSync(autd::FocalPointGain::Create(point));
	//	//autd.AppendGainSync(autd::FocalPointGain::Create(Eigen::Vector3f(10.16f * 8.5f, 10.16f * 6.5f, 528.f)));
	//}
	//foci.setZero();
	//Eigen::VectorXi amplitudes(11);
	//amplitudes.setZero();
	//amplitudes(2) = 255;
	//autd.AppendGainSync(autd::DeviceSpecificFocalPointGain::Create(foci, amplitudes));
	
	//for (int i = 0; i < autd.geometry()->numDevices(); i++) {
	//	int trans_id = i * num_trans_in_device;
	//	std::cout << "direction of AUTD #" << i << " :" << autd.geometry()->direction(trans_id).transpose() << std::endl;
	//	std::cout << "center of AUTD #" << i << ": " << autd.geometry()->position(trans_id).transpose() <<std::endl;
	//}

	std::cout << "Press any key to close " << std::endl;
	getchar();
	autd.Close();
	return 0;
}