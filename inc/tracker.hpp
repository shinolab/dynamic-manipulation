#pragma once

#include <Eigen/Dense>
#include "FloatingObject.hpp"

namespace dynaman {
	class Tracker {
	public:
		virtual ~Tracker() {};
		virtual bool open() = 0;
		virtual bool observe(DWORD& time, Eigen::Vector3f& pos, FloatingObjectPtr objPtr) = 0;
		virtual bool isOpen() = 0;
	};
}
