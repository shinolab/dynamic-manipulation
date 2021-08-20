#include <iostream>
#include <numeric>
#include <random>
#include "ThrustSearch.hpp"
#include "haptic_icon.hpp"
#include "arfModel.hpp"
#include "QPSolver.h"
#include "GainPlan.hpp"

using namespace dynaman;

int main(int argc, char** argv) {

	//Eigen::VectorXf res(1);
	//Eigen::VectorXf c(1);
	//c << -1;
	//Eigen::MatrixXf A(2, 1);
	//Eigen::VectorXf b(2);
	//A << 0.1, 0.1;
	//b << 0.05, 0.05;
	//Eigen::VectorXi condEq(2);
	//condEq << -1, -1;
	//EigenCgalLpSolver(
	//	res,
	//	A,
	//	b,
	//	c,
	//	condEq,
	//	Eigen::VectorXf::Zero(1),
	//	Eigen::VectorXf::Ones(1)
	//);

	//std::cout << res << std::endl;
	//return 0;
	auto pAupa = std::make_shared<autd::Controller>();
	haptic_icon::SetGeometry(pAupa);

	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());
	std::uniform_real_distribution dist(0.0, 250.0);
	const int num_trial = 1;
	std::vector<float> angles(num_trial);
	std::vector<float> duties(num_trial);
	const float duty_max = 1.0f;
	int num_time_slices = 1;
	auto arf_model = std::make_shared<arfModelFocusSphereExp50mm>();
	MuxThrustSearcher searcher(
		pAupa->geometry(),
		arf_model,
		num_time_slices,
		duty_max
	);

	for (int i = 0; i < angles.size(); i++) {
		auto offset = dist(engine);
		Eigen::Vector3f pos = Eigen::Vector3f::Zero();// +offset * Eigen::Vector3f::Random().normalized();
		Eigen::Vector3f direction(1, 0, 0);// = Eigen::Vector3f::Random().normalized();
		Eigen::VectorXf duty = searcher.Search(pos, direction);
		//Eigen::VectorXf duty = MaximizeThrust(pos, direction, duty_max, pAupa->geometry(), arf_model);
		Eigen::MatrixXf posRel = pos.replicate(1, pAupa->geometry()->numDevices()) - CentersAutd(pAupa->geometry());
		Eigen::Vector3f force = arf_model->arf(posRel, RotsAutd(pAupa->geometry())) * duty;
		float angle = 180.0f / pi * acosf(std::min(1.0f, force.normalized().dot(direction)));
		std::cout
			<< "Pos: " << pos.x() << ", " << pos.y() << ", " << pos.z()
			<< ", Direction: " << direction.x() << ", " << direction.y() << ", " << direction.z()
			<< ", Force: " << force.x() << ", " << force.y() << ", " << force.z()
			<< ", inner prod: " << force.normalized().dot(direction)
			<< ", angle: " << angle << " deg"
			<< ", sum(duty): " << duty.sum() << std::endl;
		angles[i] = angle;
		duties[i] = duty.sum();
	}
	
	auto mean_err_angle = std::accumulate(angles.begin(), angles.end(), 0) / angles.size();
	auto duty_sum_max = std::max_element(duties.begin(), duties.end());
	std::cout << "Average error angle: " << mean_err_angle << " [deg]" << std::endl;
	std::cout << "Max sum(duty): " << *duty_sum_max << std::endl;
	return 0;
	
}