#ifndef _DYNAMAN_THRUST_SEARCH_HPP
#define _DYNAMAN_THRUST_SEARCH_HPP

#include <Eigen/Dense>
#include "autd3.hpp"
#include "arfModel.hpp"

namespace dynaman {

	class MuxThrustSearcher {
	public:

		MuxThrustSearcher(
			autd::GeometryPtr geo,
			std::shared_ptr<arfModelLinearBase> arf_model,
			int num_time_slices,
			float duty_max = 1.0f
		);

		Eigen::VectorXf Search(
			const Eigen::Vector3f& pos,
			const Eigen::Vector3f& direction
			);

	private:
		float MaximizeThrust(
			const Eigen::Vector3f& pos,
			const Eigen::Vector3f& direction,
			const std::vector<size_t>& indexes_autd,
			Eigen::VectorXf& duty
		);
		int m_num_time_slices;
		Eigen::MatrixXf m_centers_autd;
		std::vector<Eigen::Matrix3f> m_rots_autd;
		std::shared_ptr<arfModelLinearBase> m_arf_model;
		float m_duty_max;
		std::vector<std::vector<size_t>> m_autd_comb;
	};

	Eigen::VectorXf MaximizeThrust(
		const Eigen::Vector3f& pos,
		const Eigen::Vector3f& direction,
		const float duty_max,
		autd::GeometryPtr geo,
		std::shared_ptr<arfModelLinearBase> arf_model
	);

	float MaximumThrust(
		const Eigen::Vector3f& pos,
		const Eigen::Vector3f& direction,
		const float duty_max,
		autd::GeometryPtr geo,
		std::shared_ptr<arfModelLinearBase> arfModel
	);

	float MaximumThrust(
		const Eigen::Vector3f& start,
		const Eigen::Vector3f& end,
		const float duty_max,
		const int num_points,
		autd::GeometryPtr geo,
		std::shared_ptr<arfModelLinearBase> arfModel
	);

}

#endif // !_DYNAMAN_THRUST_SEARCH_HPP
