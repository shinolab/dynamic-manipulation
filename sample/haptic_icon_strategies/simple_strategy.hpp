#ifndef _DYNAMAN_SIMPLE_STRATEGY_HPP
#define _DYNAMAN_SIMPLE_STRATEGY_HPP

#include <string>
#include <fstream>
#include <Eigen\Geometry>
#include "odcs.hpp"
#include "autd3.hpp"

#define NOMINMAX
#include "Windows.h"

namespace dynaman {
		class simple_strategy {
			Eigen::Vector3f _gainP;
			Eigen::Vector3f _gainD;
			Eigen::Vector3f _gainI;
			int _loopPeriod;
			std::string _fileName;
			float _focusBlur;
	public:
		simple_strategy(const Eigen::Vector3f& gainP,
			const Eigen::Vector3f& gainD,
			const Eigen::Vector3f& gainI,
			const int loopPeriod,
			const std::string& fileName,
			const float focusBlur);
		
		~simple_strategy() {}

		int run(dynaman::odcs& manipulator, FloatingObjectPtr objPtr, int duration);
	};
}

#endif // !_DYNAMAN_SIMPLE_STRATEGY_HPP
