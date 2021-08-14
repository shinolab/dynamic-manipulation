#ifndef _DYNAMAN_THRUST_SEARCH_HPP
#define _DYNAMAN_THRUST_SEARCH_HPP

#include <Eigen/Dense>
#include "autd3.hpp"
#include "arfModel.hpp"

namespace dynaman {

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
