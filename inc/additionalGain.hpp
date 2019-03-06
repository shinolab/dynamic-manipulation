#include <Eigen\Dense>
#include "autd3.hpp"

#define ULTRASOUND_WAVELENGTH (8.66)
#define NUM_TRANS_IN_UNIT (249)

namespace autd{
	class DeviceSpecificFocalPointGain : public Gain {

	public:
		static GainPtr Create(Eigen::MatrixXf points, Eigen::VectorXi amplitudes);
		void build();
	private:
		Eigen::MatrixXf _points;
		Eigen::VectorXi _amplitudes;
	};

	class GaussianBeamGain : public Gain {
	public:
		static GainPtr Create(Eigen::Vector3f const &point, Eigen::Vector3f const &direction, int const &amplitude, float const &beamAngle);
		void build();
	private:
		Eigen::Vector3f _point;
		Eigen::Vector3f _direction;
		int _amplitude;
		float _cosBeamAngle;
	};
}
