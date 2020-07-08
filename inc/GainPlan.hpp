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

}

#endif // !_DYNAMAN_GAINPLAN_HPP
