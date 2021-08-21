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


	auto pAupa = std::make_shared<autd::Controller>();
	haptic_icon::SetGeometry(pAupa);

	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());
	std::uniform_real_distribution dist(0.0, 100.0);
	const int num_trial = 250;
	std::vector<float> angles(num_trial);
	std::vector<float> duties(num_trial);
	std::vector<float> force_list(num_trial);
	const float duty_max = 1.0f;
	int num_time_slices = 1;
	auto arf_model = std::make_shared<arfModelFocusSphereExp50mm>();
	MuxThrustSearcher searcher1(
		pAupa->geometry(),
		arf_model,
		1,
		duty_max
	);
	MuxThrustSearcher searcher2(
		pAupa->geometry(),
		arf_model,
		2,
		duty_max
	);
	MuxThrustSearcher searcher3(
		pAupa->geometry(),
		arf_model,
		3,
		duty_max
	);
	MuxThrustSearcher searcher4(
		pAupa->geometry(),
		arf_model,
		4,
		duty_max
	);

	for (int i = 0; i < angles.size(); i++) {
		auto offset = dist(engine);
		Eigen::Vector3f pos = Eigen::Vector3f::Zero() + offset * Eigen::Vector3f::Random().normalized();
		Eigen::Vector3f direction = Eigen::Vector3f::Random().normalized();
		auto duty1 = searcher1.Search(pos, direction);
		auto duty2 = searcher2.Search(pos, direction);
		auto duty3 = searcher4.Search(pos, direction);
		auto duty4 = searcher4.Search(pos, direction);
		
		//Eigen::VectorXf duty = MaximizeThrust(pos, direction, duty_max, pAupa->geometry(), arf_model);
		Eigen::MatrixXf posRel = pos.replicate(1, pAupa->geometry()->numDevices()) - CentersAutd(pAupa->geometry());
		Eigen::Vector3f force1 = arf_model->arf(posRel, RotsAutd(pAupa->geometry())) * duty1;
		Eigen::Vector3f force2 = arf_model->arf(posRel, RotsAutd(pAupa->geometry())) * duty2;
		Eigen::Vector3f force3 = arf_model->arf(posRel, RotsAutd(pAupa->geometry())) * duty3;
		Eigen::Vector3f force4 = arf_model->arf(posRel, RotsAutd(pAupa->geometry())) * duty4;
		std::vector<Eigen::Vector3f> forces{ force1, force2, force3, force4 };
		auto itr_force = std::max_element(forces.begin(), forces.end(), [](Eigen::Vector3f& small, Eigen::Vector3f& large) {return small.norm() < large.norm(); });
		auto force = *itr_force;
		Eigen::VectorXf duty;
		switch (std::distance(forces.begin(), itr_force)) {
		case 0:
			duty = duty1;
			break;
		case 1:
			duty = duty2;
		case 2:
			duty = duty3;
		case 3:
			duty = duty4;
		}
		//duty = duty3;
		float angle = 180.0f / pi * acosf(std::min(1.0f, force.normalized().dot(direction)));
		std::cout
			<< "Pos: " << pos.x() << ", " << pos.y() << ", " << pos.z()
			<< ", Direction: " << direction.x() << ", " << direction.y() << ", " << direction.z()
			<< ", Force: " << force.x() << ", " << force.y() << ", " << force.z()
			<< ", angle: " << angle << " deg"
			<< ", sum(duty): " << duty.sum() << std::endl;
		angles[i] = angle;
		duties[i] = duty.norm();
		force_list[i] = (force.norm());
		//std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	
	auto mean_err_angle = std::accumulate(angles.begin(), angles.end(), 0) / angles.size();
	auto max_err_angle = *std::max_element(angles.begin(), angles.end());
	auto mean_force = std::accumulate(force_list.begin(), force_list.end(), 0) / force_list.size();
	auto max_force = *std::max_element(force_list.begin(), force_list.end());
	std::cout << "Average error angle: " << mean_err_angle << " deg" << std::endl;
	std::cout << "Max error angle:" << max_err_angle << " deg" << std::endl;
	std::cout << "Average force: " << mean_force << " mN" << std::endl;
	std::cout << "Max force: " << max_force << " mN" << std::endl;
	return 0;
	
}