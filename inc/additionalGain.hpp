#ifndef _ADDITIONAL_GAIN_HPP
#define _ADDITIONAL_GAIN_HPP

#include <Eigen\Dense>
#include "autd3.hpp"

#define ULTRASOUND_WAVELENGTH (8.66)
#define NUM_TRANS_IN_UNIT (249)

namespace autd {
	class DeviceSpecificFocalPointGain : public Gain {

	public:
		static GainPtr Create(Eigen::MatrixXf const& points, Eigen::VectorXi const& amplitudes);
		void build();
	private:
		Eigen::MatrixXf _points;
		Eigen::VectorXi _amplitudes;
	};

	class GaussianBeamGain : public Gain {
	public:
		static GainPtr Create(Eigen::Vector3f const& point, Eigen::Vector3f const& direction, int const& amplitude, float const& beamAngle);
		void build();
	private:
		Eigen::Vector3f _point;
		Eigen::Vector3f _direction;
		int _amplitude;
		float _cosBeamAngle;
	};

	class VortexFocalPointGain : public Gain {
	public:
		static GainPtr Create(Eigen::Matrix3Xf const& points,
			Eigen::VectorXi const& amplitudes,
			Eigen::VectorXf const& chiralities);
		void build();
	private:
		Eigen::Matrix3Xf _points;
		Eigen::VectorXi _amplitudes;
		Eigen::VectorXf _chiralities;
	};

	class ReversibleFocalPointGain : public Gain {
	public:
		static GainPtr Create(const Eigen::Vector3f& pos, uint8_t amplitude, std::function<bool(const Eigen::Vector3f&)>);
		void build();
	private:
		double PhaseShift(const Eigen::Vector3f& pos, double phase);
		Eigen::Vector3f _point;
		uint8_t _amp;
		bool _reversed;
		std::function<bool(const Eigen::Vector3f&)> _condReverse;
	};
}


#endif // !_ADDITIONAL_GAIN_HPP