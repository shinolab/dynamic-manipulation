#ifndef _DYNAMAN_BALANCE_STRATEGY_HPP
#define _DYNAMAN_BALANCE_STRAGEGY_HPP

#include <string>
#include <fstream>
#include <Eigen/Geometry>
#include "odcs.hpp"
#include "geometryUtil.hpp"
#include "additionalGain.hpp"

#define NOMINMAX
#include <Windows.h>

namespace dynaman {
	class balance_strategy {
		Eigen::Vector3f _gainP;
		Eigen::Vector3f _gainD;
		Eigen::Vector3f _gainI;
		int _loopPeriod;
		std::string _fileName;
		float _focusBlur;
	public:
		balance_strategy(
			const Eigen::Vector3f& gainP,
			const Eigen::Vector3f& gainD,
			const Eigen::Vector3f& gainI,
			const int loopPeriod,
			const std::string& fileName,
			const float focusBlur = 1.f);

		int run(dynaman::odcs& manipulator, FloatingObjectPtr objPtr, int duration);
	};
}

#endif // !_DYNAMAN_BALANCE_STRATEGY_HPP
