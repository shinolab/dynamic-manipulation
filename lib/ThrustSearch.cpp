#include "ThrustSearch.hpp"
#include "GainPlan.hpp"
#include "QPSolver.h"

using namespace dynaman;

namespace {
	auto thres_orth_norm_min = 0.01f;
	auto thres_alignment = 0.998f;
	auto thres_orth_force_min = 0.05f;
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
	int num_time_slices,
	float duty_max
) 
	:m_arf_model(arf_model),
	m_duty_max(duty_max),
	m_centers_autd(CentersAutd(geo)),
	m_rots_autd(RotsAutd(geo)),
	m_num_time_slices(num_time_slices)
{
	m_autd_comb = combination(
		geo->numDevices() - 1,
		num_time_slices
	);
}

Eigen::VectorXf MuxThrustSearcher::Search(
	const Eigen::Vector3f& pos,
	const Eigen::Vector3f& direction
) {
	const int num_cond = 4;
	Eigen::VectorXf duty_best(m_centers_autd.cols());
	duty_best.setConstant(0.0f);
	float force_best = 0.0f;
	for (auto itr_comb = m_autd_comb.begin(); itr_comb != m_autd_comb.end(); itr_comb++) {

		std::vector<int> indexes = *itr_comb;

		Eigen::MatrixXf centers_autd_used(3, m_num_time_slices);
		std::vector<Eigen::Matrix3f> rots_autd_used(m_num_time_slices);

		for (int i_used = 0; i_used < indexes.size(); i_used++) {
			centers_autd_used.col(i_used) = m_centers_autd.col(indexes[i_used]);
			rots_autd_used[i_used] = m_rots_autd[indexes[i_used]];
		}
		Eigen::MatrixXf posRel = pos.replicate(1, m_num_time_slices) - centers_autd_used;
		auto arfMat = m_arf_model->arf(posRel, rots_autd_used);
		Eigen::VectorXf c = -direction.transpose() * arfMat;
		Eigen::MatrixXf A(num_cond, m_num_time_slices);
		std::vector<Eigen::Vector3f> basis{ Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), Eigen::Vector3f::UnitZ() };
		std::sort(
			basis.begin(),
			basis.end(),
			[&direction](const Eigen::Vector3f& large, const Eigen::Vector3f& small) {
				return large.cross(direction).norm() > small.cross(direction).norm();
			});
		A.row(0) << basis[0].cross(direction).transpose() * arfMat;
		A.row(1) << basis[0].cross(direction).transpose() * arfMat;
		A.row(2) << basis[1].cross(direction).transpose() * arfMat;
		A.row(3) << basis[1].cross(direction).transpose() * arfMat;
		Eigen::VectorXf b(num_cond);
		b << thres_orth_force_min, -thres_orth_force_min, thres_orth_force_min, -thres_orth_force_min;
		Eigen::VectorXi condEq(num_cond);
		condEq << -1, 1, -1, 1;
		Eigen::VectorXf duty(m_num_time_slices);
		duty.setConstant(0.1f);
		Eigen::VectorXf lb(m_num_time_slices);
		lb.setConstant(0.f);
		auto ub = m_duty_max / m_num_time_slices * Eigen::VectorXf::Ones(m_num_time_slices);
		EigenCgalLpSolver(
			duty,
			A,
			b,
			c,
			condEq,
			lb,
			ub,
			1e-12
		);
		/*check feasibility*/
		auto force = arfMat * duty;
		if (force.normalized().dot(direction) > thres_alignment && force.norm() > force_best) {
			force_best = force.norm();
			duty_best.setZero();
			for (int i_used = 0; i_used < indexes.size(); i_used++) {
				duty_best(indexes[i_used]) = duty[i_used];
			}
			std::cout << "index: ";
			for (int i = 0; i < indexes.size(); i++) {
				std::cout << indexes[i] << ",";
			}
			std::cout << std::endl;
			std::cout << "arfMat: " << std::endl << arfMat << std::endl;
			std::cout << "A: " << std::endl << A << std::endl;
			std::cout << "b: " << b.transpose() << std::endl;
			std::cout << "c: " << c.transpose() << std::endl;
			std::cout << "lb: " << lb.transpose() << std::endl;
			std::cout << "ub: " << ub.transpose() << std::endl;
			std::cout << "duty: " << duty.transpose() << std::endl;
			//std::cout << "duty_max: " << m_duty_max << std::endl;
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