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
}
