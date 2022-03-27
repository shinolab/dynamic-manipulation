#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include "geometryUtil.hpp"
#include "privdef.hpp"


std::vector<Eigen::Vector3f> cornersWorkspaceAll(const Eigen::Vector3f& corner0, const Eigen::Vector3f& corner1) {
	std::vector<Eigen::Vector3f> corners;
	int index[3];
	for (int i = 0; i < 8; i++) { // for the number of the corners
		int res = i;
		for (int j = 0; j < 3; j++) { // for x, y, z
			index[j] = res % 2;
			res /= 2;
		}
		float x = (index[0] == 0 ? corner0.x() : corner1.x());
		float y = (index[1] == 0 ? corner0.y() : corner1.y());
		float z = (index[2] == 0 ? corner0.z() : corner1.z());
		corners.push_back(Eigen::Vector3f(x, y, z));
	}
	return corners;
}

std::vector<float> range2Points(const Eigen::Vector3f& pos_self, const Eigen::Quaternionf& quo_self, std::vector<Eigen::Vector3f> const& points) {
	std::vector<float> ranges;
	std::for_each(points.begin(), points.end(), [&ranges, &pos_self, &quo_self](Eigen::Vector3f point) {
		ranges.push_back((point - pos_self).dot(quo_self * Eigen::Vector3f::UnitZ()));
		});
	return ranges;
}

namespace dynaman {
	bool isInsideWorkspace(const Eigen::Vector3f& pos, const Eigen::Vector3f& lowerbound, const Eigen::Vector3f& upperbound) {
		Eigen::Vector3f v0 = pos - lowerbound;
		Eigen::Vector3f v1 = pos - upperbound;
		return (v0.x() * v1.x() <= 0) && (v0.y() * v1.y() <= 0) && (v0.z() * v1.z() <= 0);
	}

	Eigen::Matrix3f RotForDeviceId(int device_id, autd::GeometryPtr geo) {
		Eigen::Vector3f pos_origin = geo->position(device_id * NUM_TRANS_IN_UNIT);
		Eigen::Vector3f pos_trans_on_xaxis = geo->position(device_id * NUM_TRANS_IN_UNIT + NUM_TRANS_X - 1);
		Eigen::Vector3f pos_trans_on_yaxis = geo->position((device_id + 1) * NUM_TRANS_IN_UNIT - NUM_TRANS_X);
		Eigen::Vector3f unitX_device = (pos_trans_on_xaxis - pos_origin).normalized();
		Eigen::Vector3f unitY_device = (pos_trans_on_yaxis - pos_origin).normalized();
		Eigen::Vector3f unitZ_device = geo->direction(device_id * NUM_TRANS_IN_UNIT);
		Eigen::Matrix3f rot;
		rot << unitX_device, unitY_device, unitZ_device;
		return rot;
	}

	std::vector<Eigen::Matrix3f> RotsAutd(autd::GeometryPtr geo) {
		std::vector<Eigen::Matrix3f> rots(geo->numDevices());
		for (auto itr_device = rots.begin(); itr_device != rots.end(); itr_device++) {
			*itr_device = RotForDeviceId(std::distance(rots.begin(), itr_device), geo);
		}
		return rots;
	}

	std::vector<Eigen::Matrix3f> RotsAutd(std::shared_ptr<autd::Controller> pAupa) {
		return RotsAutd(pAupa->geometry());
	}

	Eigen::Vector3f CenterForDeviceId(int deviceId, autd::GeometryPtr geo) {
		Eigen::Vector3f center_local(TRANS_SIZE_MM * (NUM_TRANS_X / 2 - 0.5f), TRANS_SIZE_MM * (NUM_TRANS_Y / 2 - 0.5f), 0.f);
		return  RotForDeviceId(deviceId, geo) * center_local + geo->position(NUM_TRANS_IN_UNIT * deviceId);
	}

	Eigen::Matrix3Xf CentersAutd(autd::GeometryPtr geo) {
		Eigen::Matrix3Xf centers(3, geo->numDevices());
		for (int i_autd = 0; i_autd < geo->numDevices(); i_autd++) {
			centers.col(i_autd) = CenterForDeviceId(i_autd, geo);
		}
		return centers;
	}

	Eigen::Matrix3Xf DirectionsAutd(autd::GeometryPtr geo) {
		Eigen::Matrix3Xf directions;
		directions.resize(3, geo->numDevices());
		for (int i_col = 0; i_col < directions.cols(); i_col++) {
			directions.col(i_col) = geo->direction(i_col * NUM_TRANS_IN_UNIT);
		}
		return directions;
	}
}