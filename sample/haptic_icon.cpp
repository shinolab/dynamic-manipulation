#include "autd3.hpp"
#include <Eigen/Geometry>
#include <iostream>

const float pi = 3.14159265359f;
const int num_trans_in_device = 249;

namespace haptic_icon {
	int CustomAddDevice(autd::GeometryPtr geometry, const float theta, const float phi, bool isUpper) {
		const float radius = 528.f;
		const float dia_trans = 10.16f;
		Eigen::Vector3f centerLocal(dia_trans * 8.5f, dia_trans * 6.5f, 0.f);

		Eigen::Vector3f euler_angles;
		isUpper ? euler_angles << phi, theta - pi, pi / 2 : euler_angles << phi, theta - pi, -pi / 2;
		Eigen::Vector3f pos_autd = radius * Eigen::Vector3f(sinf(theta) * cosf(phi), sinf(theta) * cosf(phi), cosf(theta))
			- Eigen::AngleAxisf(euler_angles.x(), Eigen::Vector3f::UnitZ())
			* Eigen::AngleAxisf(euler_angles.y(), Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(euler_angles.z(), Eigen::Vector3f::UnitZ()) * centerLocal;
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
	const float theta0 = 0.f, phi0 = pi / 4;
	//upper autd
	const float theta1 = pi / 4, phi1 = pi / 4;
	const float theta2 = pi / 4, phi2 = 2 * pi / 4;
	const float theta3 = pi / 4, phi3 = 3 * pi / 4;
	const float theta4 = pi / 4, phi4 = 4 * pi / 4;
	//middle autd
	const float theta5 = pi / 2, phi5 = pi / 2;
	const float theta6 = pi / 2, phi6 = 3 * pi / 2;
	//lower autd
	const float theta7 = 3 * pi / 4, phi7 = pi / 4;
	const float theta8 = 3 * pi / 4, phi8 = 2 * pi / 4;
	const float theta9 = 3 * pi / 4, phi9 = 3 * pi / 4;
	const float theta10 = 3 * pi / 4, phi10 = 4 * pi / 4;
	
	haptic_icon::CustomAddDevice(autd.geometry(), theta0, phi0, false);
	haptic_icon::CustomAddDevice(autd.geometry(), theta1, phi1, false);
	haptic_icon::CustomAddDevice(autd.geometry(), theta2, phi2, false);
	haptic_icon::CustomAddDevice(autd.geometry(), theta3, phi3, false);
	haptic_icon::CustomAddDevice(autd.geometry(), theta4, phi4, false);
	haptic_icon::CustomAddDevice(autd.geometry(), theta5, phi5, false);
	haptic_icon::CustomAddDevice(autd.geometry(), theta6, phi6, false);
	haptic_icon::CustomAddDevice(autd.geometry(), theta7, phi7, false);
	haptic_icon::CustomAddDevice(autd.geometry(), theta8, phi8, false);
	haptic_icon::CustomAddDevice(autd.geometry(), theta9, phi9, false);
	haptic_icon::CustomAddDevice(autd.geometry(), theta10, phi10, false);

	for (int i = 0; i < autd.geometry()->numDevices(); i++) {
		int trans_id = i * num_trans_in_device;
		std::cout << "direction of AUTD " << i << ":" << autd.geometry()->direction(trans_id).transpose() << std::endl;
	}
	autd.AppendGainSync(autd::FocalPointGain::Create(Eigen::Vector3f::Zero()));
	autd.AppendModulationSync(autd::SineModulation::Create(150));
	autd.Close();
}