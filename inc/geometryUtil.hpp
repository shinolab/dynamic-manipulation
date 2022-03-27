#pragma once
#include <Eigen/Dense>
#include <vector>
#include <autd3.hpp>

std::vector<Eigen::Vector3f> cornersWorkspaceAll(const Eigen::Vector3f& corner0, const Eigen::Vector3f& corner1);

std::vector<float> range2Points(const Eigen::Vector3f & pos_self, const Eigen::Quaternionf &quo_self, std::vector<Eigen::Vector3f> const &points);

namespace dynaman {

	bool isInsideWorkspace(const Eigen::Vector3f& pos, const Eigen::Vector3f& lowerbound, const Eigen::Vector3f& upperbound);

	Eigen::Matrix3f RotForDeviceId(int deviceId, autd::GeometryPtr geo);

	std::vector<Eigen::Matrix3f> RotsAutd(autd::GeometryPtr geo);

	std::vector<Eigen::Matrix3f> RotsAutd(std::shared_ptr<autd::Controller> pAupa);

	Eigen::Vector3f CenterForDeviceId(int deviceId, autd::GeometryPtr geo);

	Eigen::Matrix3Xf CentersAutd(autd::GeometryPtr geo);

	Eigen::Matrix3Xf DirectionsAutd(autd::GeometryPtr geo);
}