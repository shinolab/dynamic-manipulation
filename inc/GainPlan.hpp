#ifndef _DYNAMAN_GAINPLAN_HPP
#define _DYNAMAN_GAINPLAN_HPP

#include <Eigen/Geometry>
#include "autd3.hpp"
#include "privdef.hpp"

namespace dynaman {

	Eigen::Matrix3f rotForDeviceId(int deviceId, autd::GeometryPtr geo);

	Eigen::Vector3f centerForDeviceId(int deviceId, autd::GeometryPtr geo);

	void centersAutd(autd::GeometryPtr geo, Eigen::Matrix3f& posRel);

	void directionsAutd(autd::GeometryPtr geo, Eigen::Matrix3f& directions);
	
	Eigen::VectorXf FindDutyQpMultiplex(
		const Eigen::Matrix3Xf& forceMat,
		const Eigen::Vector3f& forceTarget
	);

}

#endif // !_DYNAMAN_GAINPLAN_HPP
