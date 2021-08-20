#include "ThrustSearch.hpp"
#include "GainPlan.hpp"
#include "QPSolver.h"

using namespace dynaman;

namespace {
	constexpr int NUM_TIME_SLICES = 4;
	auto thres_orth_norm_min = 0.01f;
	auto thres_alignment = 0.998f;
}

std::vector<std::vector<int>> combination(int max, int num) {
	if (num == 1) {
		std::vector<std::vector<int>> v;
		for (int i = 0; i <= max; i++) {
			v.push_back({ i });
		}
		return v;
	}
	if (num == max + 1) {
		std::vector<int> v;
		for (int i = 0; i <= max; i++) {
			v.push_back(i);
		}
		return { v };
	}
	auto others = combination(max - 1, num);
	auto inc = combination(max - 1, num - 1);
	for (auto itr = inc.begin(); itr != inc.end(); itr++) {
		itr->push_back(max);
	}
	inc.insert(inc.end(), others.begin(), others.end());
	return inc;
}

MuxThrustSearcher::MuxThrustSearcher(
	autd::GeometryPtr geo,
	std::shared_ptr<arfModelLinearBase> arf_model,
	float duty_max
) 
	:m_arf_model(arf_model),
	m_duty_max(duty_max),
	m_centers_autd(CentersAutd(geo)),
	m_rots_autd(RotsAutd(geo))
{
	m_autd_comb = combination(
		geo->numDevices() - 1,
		NUM_TIME_SLICES
	);
}

Eigen::VectorXf MuxThrustSearcher::Search(
	const Eigen::Vector3f& pos,
	const Eigen::Vector3f& direction
) {
	const int num_cond = 2;
	Eigen::Vector3f orth_x = Eigen::Vector3f::UnitX().cross(direction);
	Eigen::Vector3f orth_y = Eigen::Vector3f::UnitY().cross(direction);
	Eigen::Vector3f orth_z = Eigen::Vector3f::UnitZ().cross(direction);
	Eigen::VectorXf duty_best(m_centers_autd.cols());
	duty_best.setConstant(0.0f);
	float force_best = 0.0f;
	for (auto itr_comb = m_autd_comb.begin(); itr_comb != m_autd_comb.end(); itr_comb++) {

		auto idx_1 = (*itr_comb)[0];
		auto idx_2 = (*itr_comb)[1];
		auto idx_3 = (*itr_comb)[2];
		auto idx_4 = (*itr_comb)[3];

		Eigen::MatrixXf centers_autd_used(3, NUM_TIME_SLICES);
		centers_autd_used << m_centers_autd.col(idx_1), m_centers_autd.col(idx_2), m_centers_autd.col(idx_3), m_centers_autd.col(idx_4);
		std::vector<Eigen::Matrix3f> rots_autd_used{ m_rots_autd[idx_1], m_rots_autd[idx_2], m_rots_autd[idx_3], m_rots_autd[idx_4] };
		Eigen::MatrixXf posRel = pos.replicate(1, NUM_TIME_SLICES) - centers_autd_used;
		auto arfMat = m_arf_model->arf(posRel, rots_autd_used);
		Eigen::VectorXf c = -direction.transpose() * arfMat;
		Eigen::MatrixXf A(num_cond, NUM_TIME_SLICES);
		std::vector<Eigen::Vector3f> basis{ Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), Eigen::Vector3f::UnitZ() };
		std::sort(
			basis.begin(),
			basis.end(),
			[&direction](const Eigen::Vector3f& large, const Eigen::Vector3f& small) {
				return large.cross(direction).norm() > small.cross(direction).norm();
			});
		A.row(0) << basis[0].cross(direction).transpose() * arfMat;
		A.row(1) << basis[1].cross(direction).transpose() * arfMat;
		Eigen::VectorXf b(num_cond);
		b << 0, 0;
		Eigen::VectorXi condEq(num_cond);
		condEq << 0, 0;
		Eigen::VectorXf duty(NUM_TIME_SLICES);
		duty.setConstant(0.1f);
		Eigen::VectorXf lb(NUM_TIME_SLICES);
		lb.setConstant(0.f);
		auto ub = 0.25 * Eigen::VectorXf::Ones(NUM_TIME_SLICES);
		EigenCgalLpSolver(
			duty,
			A,
			b,
			c,
			condEq,
			lb,
			ub,
			1e-4f
		);
		/*check feasibility*/
		std::cout << "index: " << idx_1 << "," << idx_2 << "," << idx_3 << ", " << idx_4 << std::endl;
		std::cout << "arfMat: " << std::endl << arfMat << std::endl;

		std::cout << "A: " << std::endl << A << std::endl;
		std::cout << "b: " << b.transpose() << std::endl;
		std::cout << "c: " << c.transpose() << std::endl;
		std::cout << "lb: " << lb.transpose() << std::endl;
		std::cout << "ub: " << ub.transpose() << std::endl;
		std::cout << "duty: "  << duty.transpose() << std::endl;
		std::cout << "duty_max: " << m_duty_max << std::endl;
		auto force = arfMat * duty;
		if (force.normalized().dot(direction) > thres_alignment && force.norm() > force_best) {
			duty_best.setZero();
			duty_best(idx_1) = duty[0];
			duty_best(idx_2) = duty[1];
			duty_best(idx_3) = duty[2];
		}	
	}
	return duty_best;
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
	return (arf_model->arf(posRel, RotsAutd(geo)) * duty).norm();
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