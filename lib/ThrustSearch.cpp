#include "ThrustSearch.hpp"
#include "GainPlan.hpp"
#include "QPSolver.h"

namespace {
	auto thres_orth_norm_min = 0.01f;
}

Eigen::VectorXf dynaman::MaximizeThrust(
	const Eigen::Vector3f& pos,
	const Eigen::Vector3f& direction,
	const float duty_max,
	autd::GeometryPtr geo,
	std::shared_ptr<arfModelLinearBase> arf_model
) {
	const int num_cond = 3;
	Eigen::Vector3f orth_x = Eigen::Vector3f::UnitX().cross(direction);
	Eigen::Vector3f orth_y = Eigen::Vector3f::UnitY().cross(direction);
	Eigen::Vector3f orth_z = Eigen::Vector3f::UnitZ().cross(direction);
	Eigen::MatrixXf posRel = pos.replicate(1, geo->numDevices()) - CentersAutd(geo);
	Eigen::VectorXf duty;
	auto arfMat = arf_model->arf(posRel, RotsAutd(geo));
	Eigen::VectorXf c = -direction.transpose() * arfMat;
	Eigen::MatrixXf A(num_cond, geo->numDevices());
	A.row(0) << Eigen::RowVectorXf::Ones(geo->numDevices());
	A.row(1) << (orth_x.norm() > thres_orth_norm_min ? orth_x.transpose() * arfMat : orth_y.transpose() * arfMat);
	A.row(2) << (orth_y.norm() > thres_orth_norm_min ? orth_y.transpose() * arfMat : orth_z.transpose() * arfMat);
	Eigen::VectorXf b(num_cond);
	b << duty_max, 0, 0;
	//std::cout
	//	<< "A: " << std::endl << A
	//	<< std::endl << "b: " << std::endl << b.transpose() << std::endl;
	//std::cout << "arfMat: " << std::endl << arfMat << std::endl;
	//std::cout << "c:" << std::endl << c.transpose() << std::endl;
	Eigen::VectorXi condEq(num_cond);
	condEq << -1, 0, 0;
	EigenCgalLpSolver(
		duty,
		A,
		b,
		c,
		condEq,
		Eigen::VectorXf::Zero(geo->numDevices()),
		1.001 * Eigen::VectorXf::Ones(geo->numDevices())
	);
	return duty;
}

float dynaman::MaximumThrust(
	const Eigen::Vector3f& pos,
	const Eigen::Vector3f& direction,
	const float duty_max,
	autd::GeometryPtr geo,
	std::shared_ptr<arfModelLinearBase> arf_model
) {
	Eigen::MatrixXf posRel = pos.replicate(1, geo->numDevices()) - CentersAutd(geo);

	Eigen::VectorXf duty = MaximizeThrust(pos, direction, duty_max, geo, arf_model);
	return (arf_model->arf(posRel, RotsAutd(geo)) * duty.norm()).norm();
}

float dynaman::MaximumThrust(
	const Eigen::Vector3f& start,
	const Eigen::Vector3f& end,
	const float duty_max,
	const int num_points,
	autd::GeometryPtr geo,
	std::shared_ptr<arfModelLinearBase> arf_model
) {
	auto len_path = (end - start).norm();
	auto direction = (end - start).normalized();
	auto interval = len_path / (num_points - 1);
	float thrust_min = FLT_MAX;
	for (int i_point = 0; i_point < len_path; i_point++) {
		auto pos = start + interval * direction;
		auto thrust = MaximumThrust(pos, direction, duty_max, geo, arf_model);
		if (thrust < thrust_min) {
			thrust_min = thrust;
		}
	}
	return thrust_min;
}