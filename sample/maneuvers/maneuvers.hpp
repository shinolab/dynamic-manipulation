#ifndef _DYNAMAN_MANEUVERS_HPP
#define _DYNAMAN_MANEUVERS_HPP

#include <Eigen/Geometry>
#include <thread>
#include <chrono>
#include "odcs.hpp"
#include <iostream>

namespace {
	const float pi = 3.14159265359f;
}

namespace dynaman {

	class linear_scan {
		Eigen::Vector3f _direction;
		Eigen::Vector3f _pos_start;
		float _dist_step;
		int _num_step;
		int _wait_seconds;
	public:
		linear_scan(const Eigen::Vector3f& direction,
			const Eigen::Vector3f& pos_start = Eigen::Vector3f(0, 0, 0),
			float dist_step = 50.f,
			int num_step = 10,
			int wait_seconds = 60);

		~linear_scan() = default;

		void run(dynaman::FloatingObjectPtr objPtr);

	};

	class linear_maneuver_rep {
		Eigen::Vector3f _pos_start;
		Eigen::Vector3f _pos_end;
		float _v_init;
		float _delta_v;
		int _num_step;

	public:
		linear_maneuver_rep(const Eigen::Vector3f& pos_start, const Eigen::Vector3f& pos_end, float v_init = 200, float delta_v = 100, int num_step = 20);

		void run(dynaman::FloatingObjectPtr objPtr);
	};

	class circular_maneuver {
		Eigen::Vector3f _center;
		float _radius;
		float _raan;
		float _inclination;
		float _vel_init;
		float _vel_delta;
		float _phase_init;
		int _num_step;
	public:
		circular_maneuver(float radius,
			const Eigen::Vector3f& center = Eigen::Vector3f(0, 0, 0),
			float inclination = pi / 2.f,
			float raan = 0.f,
			float vel_init = 50.f,
			float vel_delta = 25.f,
			int num_step = 81,
			float phase_init = pi / 2.f);

		void run(dynaman::FloatingObjectPtr objPtr);

	};
}

#endif // !_DYNAMAN_MANEUVERS_HPP
