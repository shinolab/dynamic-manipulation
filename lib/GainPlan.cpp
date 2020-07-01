#include "GainPlan.hpp"

namespace dynaman {
	
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

	Eigen::Vector3f CenterForDeviceId(int deviceId, autd::GeometryPtr geo) {
		Eigen::Vector3f center_local(TRANS_SIZE_MM * NUM_TRANS_X / 2, TRANS_SIZE_MM * NUM_TRANS_Y / 2, 0.f);
		return  RotForDeviceId(deviceId, geo) * center_local + geo->position(NUM_TRANS_IN_UNIT * deviceId);
	}

	Eigen::Matrix3Xf CentersAutd(autd::GeometryPtr geo) {
		Eigen::Matrix3Xf centers;
		centers.resize(3, geo->numDevices());
		for (int i_autd = 0; i_autd < geo->numDevices(); i_autd++){ 
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