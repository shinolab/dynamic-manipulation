#include "K4aHeadTracker.hpp"


K4aHeadTracker::K4aHeadTracker(
	const Eigen::Vector3f& pos_device,
	const Eigen::Matrix3f& rot_device
) 
	:m_affine_device(Eigen::Translation3f(pos_device) * rot_device){}

std::shared_ptr<K4aHeadTracker> K4aHeadTracker::Create(const Eigen::Vector3f& pos_device, const Eigen::Matrix3f& rot_device) {
	return std::make_shared<K4aHeadTracker>(pos_device, rot_device);
}

void K4aHeadTracker::Open() {
	m_device.open(K4A_DEVICE_DEFAULT);

	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
	deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;

	m_device.start_cameras(&deviceConfig);

	k4a::calibration sensorCalibration = m_device.get_calibration(deviceConfig.depth_mode, deviceConfig.color_resolution);
	m_tracker = k4abt::tracker::create(sensorCalibration);
}

void K4aHeadTracker::Close() {
	m_tracker.shutdown();
	m_tracker.destroy();
	m_device.stop_cameras();
	m_device.close();
}

bool K4aHeadTracker::GetHeadGeometry(Eigen::Vector3f& pos, Eigen::Quaternionf& quo) {
	k4a::capture cap;
	bool captured = m_device.get_capture(&cap, std::chrono::milliseconds(100));
	if (!captured) {
		return false;
	}
	bool tracked = m_tracker.enqueue_capture(cap, std::chrono::milliseconds(100));
	if (!tracked) {
		return false;
	}
	k4abt::frame bodyFrame;
	auto frame_popped = m_tracker.pop_result(&bodyFrame, std::chrono::milliseconds(100));
	if (!frame_popped) {
		return false;
	}
	auto num_bodies = bodyFrame.get_num_bodies();
	if (num_bodies == 0) {
		return false;
	}
	auto headJoint = bodyFrame.get_body(0).skeleton.joints[K4ABT_JOINT_HEAD];
	auto posArray = headJoint.position.v;
	auto quoArray = bodyFrame.get_body(0).skeleton.joints[K4ABT_JOINT_HEAD].orientation.v;
	pos = Eigen::Map<Eigen::Vector3f>(posArray);
	quo = Eigen::Map<Eigen::Quaternionf>(quoArray);
	return true;
}


bool K4aHeadTracker::TransformHead2Global(const Eigen::Vector3f& pos_in_head, Eigen::Vector3f& pos_in_global) {
	Eigen::Vector3f posHead;
	Eigen::Quaternionf quoHead;
	bool tracked = GetHeadGeometry(posHead, quoHead);
	if (!tracked)
	{
		return false;
	}
	auto pos_in_device = (Eigen::Translation3f(posHead) * quoHead) * pos_in_head;
	pos_in_global = m_affine_device * pos_in_device;
	return true;
}

bool K4aHeadTracker::TransformGlobal2Head(const Eigen::Vector3f& pos_in_global, Eigen::Vector3f& pos_in_head) {
	Eigen::Vector3f posHead;
	Eigen::Quaternionf quoHead;
	bool tracked = GetHeadGeometry(posHead, quoHead);
	if (!tracked) {
		return false;
	}
	auto pos_in_device = m_affine_device.inverse() * pos_in_global;
	pos_in_head = (Eigen::Translation3f(posHead) * quoHead).inverse() * pos_in_device;
	return true;
}