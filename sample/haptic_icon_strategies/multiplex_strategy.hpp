#ifndef _DYNAMAN_MULTIPLEX_STRATEGY_HPP
#define _DYNAMAN_MULTIPLEX_STRATEGY_HPP

#include <string>
#include <Eigen/Geometry>
#include "odcs.hpp"

namespace dynaman{

	class multiplex_strategy {
		Eigen::Vector3f _gainP;
		Eigen::Vector3f _gainD;
		Eigen::Vector3f _gainI;
		std::string _fileName;
		int _loopPeriod;
		float _freq;
		float _lambda;

	public:
		multiplex_strategy(
			const Eigen::Vector3f& gainP,
			const Eigen::Vector3f& gainD,
			const Eigen::Vector3f& gainI,
			int loopPeriod,
			const std::string& fileName,
			int freq,
			float lambda);

		int run(dynaman::odcs& manipulator, FloatingObjectPtr objPtr, int duration);
	};

}

#endif // !_DYNAMAN_MULTIPLEX_STRATEGY_HPP
