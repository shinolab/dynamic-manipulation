#include "GainPlan.hpp"

namespace dynaman {

	Eigen::Matrix3f rotForDeviceId(int device_id, autd::GeometryPtr geo) {
		auto pos_origin = geo->position(device_id * NUM_TRANS_IN_UNIT);
		auto pos_x = geo->position(device_id * NUM_TRANS_IN_UNIT + NUM_TRANS_X - 1);
		auto pos_y = geo->position((device_id + 1) * NUM_TRANS_IN_UNIT - NUM_TRANS_X);
		auto unitX_device = (pos_x - pos_origin).normalized();
		auto unitY_device = (pos_y - pos_origin).normalized();
		auto unitZ_device = geo->direction(device_id * NUM_TRANS_IN_UNIT);
		Eigen::Matrix3f rot;
		rot << unitX_device, unitY_device, unitZ_device;
		return rot;
	}

	void centersAutd(autd::GeometryPtr geo, Eigen::Matrix3f& posRel) {
		posRel.resize(3, geo->numDevices());
		for (int i_autd = 0; i_autd < geo->numDevices(); i_autd++) {
			Eigen::Vector3f center_local(TRANS_SIZE_MM * NUM_TRANS_X / 2, TRANS_SIZE_MM * NUM_TRANS_Y / 2, 0.f);
			Eigen::Vector3f center_global = rotForDeviceId(i_autd, geo) * center_local + geo->position(NUM_TRANS_IN_UNIT * i_autd);
			posRel.col(i_autd) = center_global;
		}
	}

	void directionsAutd(autd::GeometryPtr geo, Eigen::Matrix3f& directions) {
		directions.resize(geo->numDevices());
		for (int i_col = 0; i_col < directions.cols(); i_col++) {
			directions.col(i_col) = geo->direction(i_col * NUM_TRANS_IN_UNIT);
		}
	}

}