#include "maneuvers.hpp"
#include <Eigen/Geometry>

namespace dynaman {
	linear_scan::linear_scan(const Eigen::Vector3f& direction,
		const Eigen::Vector3f& pos_start,
		float dist_step,
		int num_step,
		int wait_seconds)
		:_direction(direction),
		_pos_start(pos_start),
		_dist_step(dist_step),
		_num_step(num_step),
		_wait_seconds(wait_seconds) {}

	void linear_scan::run(FloatingObjectPtr objPtr) {
		for (int i_step = 0; i_step < _num_step; i_step++) {
			Eigen::Vector3f pos_tgt = _pos_start + i_step * _dist_step * _direction;
			std::cout << "updating target position: " << pos_tgt.transpose() << std::endl;
			objPtr->updateStatesTarget(pos_tgt, Eigen::Vector3f(0, 0, 0));
			std::this_thread::sleep_for(std::chrono::seconds(_wait_seconds));
		}
	}

	linear_maneuver_rep::linear_maneuver_rep(const Eigen::Vector3f& pos_start, const Eigen::Vector3f& pos_end, float v_init, float delta_v, int num_step)
		:_pos_start(pos_start), _pos_end(pos_end), _v_init(v_init), _delta_v(delta_v), _num_step(num_step) {}

	void linear_maneuver_rep::run(dynaman::FloatingObjectPtr objPtr) {
		float time_init = 4 * (_pos_end - _pos_start).norm() / _v_init;
		std::this_thread::sleep_for(std::chrono::seconds(20));
		objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(10, timeGetTime() / 1000.f, objPtr->getPosition(), _pos_start));
		for (int i_step = 0; i_step < _num_step; i_step++) {
			float v_max = _v_init + i_step * _delta_v;
			float time_trans = _v_init / v_max * time_init;
			std::cout << "target velocity: " << v_max << "[mm/s]" << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(20));
			objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(time_trans, timeGetTime() / 1000.f, _pos_start, _pos_end));
			std::this_thread::sleep_for(std::chrono::seconds(20));
			objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(time_trans, timeGetTime() / 1000.f, _pos_end, _pos_start));
		}
	}

	circular_maneuver::circular_maneuver(float radius,
		const Eigen::Vector3f& center,
		float inclination,
		float raan,
		float vel_init,
		float vel_delta,
		int num_step,
		float phase_init)
		:_center(center),
		_radius(radius),
		_inclination(inclination),
		_raan(raan),
		_vel_init(vel_init),
		_vel_delta(vel_delta),
		_num_step(num_step),
		_phase_init(phase_init) {}

	void circular_maneuver::run(dynaman::FloatingObjectPtr objPtr) {
		std::this_thread::sleep_for(std::chrono::seconds(20));
		for (int i_step = 0; i_step < _num_step; i_step++) {
			float vel = _vel_init + i_step * _vel_delta;
			float period = 2.0f * _radius * pi / vel;
			std::cout << "target velocity: " << vel << "[mm/s] (period " << period << "[s])" << std::endl;
			objPtr->SetTrajectory(
				dynaman::TrajectoryCircle::Create(
					_center,
					_radius,
					_inclination,
					_raan,
					period,
					0.f,
					timeGetTime() / 1000.f
				)
			);
			std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(period * 1000)));
		}
	}
}