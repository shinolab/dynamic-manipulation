#ifndef _HAPTIC_ICON_HPP
#define _HAPTIC_ICON_HPP

#include "odcs.hpp"
#include <Eigen/Geometry>

namespace {
	const float pi = 3.14159265359f;
	const int num_trans_in_device = 249;
	const float radius_ws = 528.f;
	const float dia_trans = 10.16f;
}

namespace haptic_icon {

	Eigen::Vector3f Polar2Euler(const float theta, const float phi, bool isUpper) {
		return (isUpper ? Eigen::Vector3f(phi, theta - pi, pi / 2.f) : Eigen::Vector3f(phi, theta - pi, -pi / 2.f));
	}

	Eigen::Vector3f Polar2Position(const float theta, const float phi, bool isUpper) {
		Eigen::Vector3f euler_angles = Polar2Euler(theta, phi, isUpper);
		Eigen::Vector3f centerLocal(dia_trans * 8.5f, dia_trans * 6.5f, 0.f);
		Eigen::Vector3f centerGlobal = radius_ws * Eigen::Vector3f(sinf(theta) * cosf(phi), sinf(theta) * sinf(phi), cosf(theta));
		return centerGlobal
			- Eigen::AngleAxisf(euler_angles.x(), Eigen::Vector3f::UnitZ())
			* Eigen::AngleAxisf(euler_angles.y(), Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(euler_angles.z(), Eigen::Vector3f::UnitZ()) * centerLocal;
	}

	void SetGeometry(std::shared_ptr<autd::Controller> pAupa) {
		const float theta0 = 0.f, phi0 = -3 * pi / 4; bool isUpper0 = true;
		//upper autd
		const float theta1 = pi / 4, phi1 = pi / 4; bool isUpper1 = true;
		const float theta2 = pi / 4, phi2 = 3 * pi / 4; bool isUpper2 = true;
		const float theta3 = pi / 4, phi3 = -3 * pi / 4; bool isUpper3 = true;
		const float theta4 = pi / 4, phi4 = -pi / 4; bool isUpper4 = true;
		//middle autd
		const float theta5 = pi / 2, phi5 = pi / 2; bool isUpper5 = true;
		const float theta6 = pi / 2, phi6 = 3 * pi / 2; bool isUpper6 = true;
		//lower autd
		const float theta7 = 3 * pi / 4, phi7 = pi / 4; bool isUpper7 = false;
		const float theta8 = 3 * pi / 4, phi8 = 3 * pi / 4; bool isUpper8 = false;
		const float theta9 = 3 * pi / 4, phi9 = -3 * pi / 4; bool isUpper9 = false;
		const float theta10 = 3 * pi / 4, phi10 = -pi / 4; bool isUpper10 = false;
		pAupa->geometry()->AddDevice(Polar2Position(theta9, phi9, isUpper9), Polar2Euler(theta9, phi9, isUpper9), 0);
		pAupa->geometry()->AddDevice(Eigen::Vector3f(-528.f, 10.16f * 6.5f, -10.16f * 8.5f), Eigen::Vector3f(0, pi / 2, pi), 1);
		pAupa->geometry()->AddDevice(Polar2Position(theta8, phi8, isUpper8), Polar2Euler(theta8, phi8, isUpper8), 2);
		pAupa->geometry()->AddDevice(Polar2Position(theta7, phi7, isUpper7), Polar2Euler(theta7, phi7, isUpper7), 3);
		pAupa->geometry()->AddDevice(Eigen::Vector3f(528.f, -10.16f * 6.5f, -10.16f * 8.5f), Eigen::Vector3f(0, -pi / 2, 0), 4);
		pAupa->geometry()->AddDevice(Polar2Position(theta10, phi10, isUpper10), Polar2Euler(theta10, phi10, isUpper10), 5);
		pAupa->geometry()->AddDevice(Polar2Position(theta4, phi4, isUpper4), Polar2Euler(theta4, phi4, isUpper4), 6);
		pAupa->geometry()->AddDevice(Polar2Position(theta1, phi1, isUpper1), Polar2Euler(theta1, phi1, isUpper1), 7);
		pAupa->geometry()->AddDevice(Polar2Position(theta2, phi2, isUpper2), Polar2Euler(theta2, phi2, isUpper2), 8);
		pAupa->geometry()->AddDevice(Polar2Position(theta3, phi3, isUpper3), Polar2Euler(theta3, phi3, isUpper3), 9);
		pAupa->geometry()->AddDevice(Polar2Position(theta0, phi0, isUpper0), Polar2Euler(theta0, phi0, isUpper0), 10);
	}

	void Initialize(std::shared_ptr<autd::Controller> pAupa) {

		pAupa->Open(autd::LinkType::ETHERCAT);
		SetGeometry(pAupa);
	}

	void SetGeometry(dynaman::odcs& manipulator) {
		//top autd
		const float theta0 = 0.f, phi0 = -3 * pi / 4; bool isUpper0 = true;
		//upper autd
		const float theta1 = pi / 4, phi1 = pi / 4; bool isUpper1 = true;
		const float theta2 = pi / 4, phi2 = 3 * pi / 4; bool isUpper2 = true;
		const float theta3 = pi / 4, phi3 = -3 * pi / 4; bool isUpper3 = true;
		const float theta4 = pi / 4, phi4 = -pi / 4; bool isUpper4 = true;
		//middle autd
		const float theta5 = pi / 2, phi5 = pi / 2; bool isUpper5 = true;
		const float theta6 = pi / 2, phi6 = 3 * pi / 2; bool isUpper6 = true;
		//lower autd
		const float theta7 = 3 * pi / 4, phi7 = pi / 4; bool isUpper7 = false;
		const float theta8 = 3 * pi / 4, phi8 = 3 * pi / 4; bool isUpper8 = false;
		const float theta9 = 3 * pi / 4, phi9 = -3 * pi / 4; bool isUpper9 = false;
		const float theta10 = 3 * pi / 4, phi10 = -pi / 4; bool isUpper10 = false;
		manipulator.AddDevice(Polar2Position(theta9, phi9, isUpper9), Polar2Euler(theta9, phi9, isUpper9), 0);
		manipulator.AddDevice(Eigen::Vector3f(-528.f, 10.16f * 6.5f, -10.16f * 8.5f), Eigen::Vector3f(0, pi / 2, pi), 1);
		manipulator.AddDevice(Polar2Position(theta8, phi8, isUpper8), Polar2Euler(theta8, phi8, isUpper8), 2);
		manipulator.AddDevice(Polar2Position(theta7, phi7, isUpper7), Polar2Euler(theta7, phi7, isUpper7), 3);
		manipulator.AddDevice(Eigen::Vector3f(528.f, -10.16f * 6.5f, -10.16f * 8.5f), Eigen::Vector3f(0, -pi / 2, 0), 4);
		manipulator.AddDevice(Polar2Position(theta10, phi10, isUpper10), Polar2Euler(theta10, phi10, isUpper10), 5);
		manipulator.AddDevice(Polar2Position(theta4, phi4, isUpper4), Polar2Euler(theta4, phi4, isUpper4), 6);
		manipulator.AddDevice(Polar2Position(theta1, phi1, isUpper1), Polar2Euler(theta1, phi1, isUpper1), 7);
		manipulator.AddDevice(Polar2Position(theta2, phi2, isUpper2), Polar2Euler(theta2, phi2, isUpper2), 8);
		manipulator.AddDevice(Polar2Position(theta3, phi3, isUpper3), Polar2Euler(theta3, phi3, isUpper3), 9);
		manipulator.AddDevice(Polar2Position(theta0, phi0, isUpper0), Polar2Euler(theta0, phi0, isUpper0), 10);
	}

	void Initialize(dynaman::odcs& manipulator) {
		manipulator.Initialize();
		SetGeometry(manipulator);
	}

	void InitializeLower(dynaman::odcs& manipulator) {
		//middle autd
		const float theta5 = pi / 2, phi5 = pi / 2; bool isUpper5 = true;
		const float theta6 = pi / 2, phi6 = 3 * pi / 2; bool isUpper6 = true;
		//lower autd
		const float theta7 = 3 * pi / 4, phi7 = pi / 4; bool isUpper7 = false;
		const float theta8 = 3 * pi / 4, phi8 = 3 * pi / 4; bool isUpper8 = false;
		const float theta9 = 3 * pi / 4, phi9 = -3 * pi / 4; bool isUpper9 = false;
		const float theta10 = 3 * pi / 4, phi10 = -pi / 4; bool isUpper10 = false;
		manipulator.Initialize();
		manipulator.AddDevice(Polar2Position(theta9, phi9, isUpper9), Polar2Euler(theta9, phi9, isUpper9), 0);
		manipulator.AddDevice(Eigen::Vector3f(-528.f, 10.16f * 6.5f, -10.16f * 8.5f), Eigen::Vector3f(0, pi / 2, pi), 1);
		manipulator.AddDevice(Polar2Position(theta8, phi8, isUpper8), Polar2Euler(theta8, phi8, isUpper8), 2);
		manipulator.AddDevice(Polar2Position(theta7, phi7, isUpper7), Polar2Euler(theta7, phi7, isUpper7), 3);
		manipulator.AddDevice(Eigen::Vector3f(528.f, -10.16f * 6.5f, -10.16f * 8.5f), Eigen::Vector3f(0, -pi / 2, 0), 4);
		manipulator.AddDevice(Polar2Position(theta10, phi10, isUpper10), Polar2Euler(theta10, phi10, isUpper10), 5);
	}
}

#endif // !_HAPTIC_ICON_HPP

