#ifndef _DYNAMAN_GAINPLAN_HPP
#define _DYNAMAN_GAINPLAN_HPP

#include <Eigen/Geometry>
#include "autd3.hpp"
#include "privdef.hpp"

namespace dynaman {

	Eigen::Matrix3f RotForDeviceId(int deviceId, autd::GeometryPtr geo);

	std::vector<Eigen::Matrix3f> RotsAutd(autd::GeometryPtr geo);

	Eigen::Vector3f CenterForDeviceId(int deviceId, autd::GeometryPtr geo);

	Eigen::Matrix3Xf CentersAutd(autd::GeometryPtr geo);

	Eigen::Matrix3Xf DirectionsAutd(autd::GeometryPtr geo);

	inline Eigen::Matrix3Xf posRel(
		const Eigen::Vector3f postiion,
		const std::shared_ptr<const autd::Controller> pAupa
	);

	inline Eigen::Matrix3Xf directionsRel(const Eigen::Vector3f position, const std::shared_ptr<const autd::Controller> pAupa);

}

#endif // !_DYNAMAN_GAINPLAN_HPP
