#include <Eigen\Dense>
#include <boost\assert.hpp>
#include "autd3.hpp"
#include "additionalGain.hpp"

autd::GainPtr autd::DeviceSpecificFocalPointGain::Create(Eigen::MatrixXf points, Eigen::VectorXi amplitudes)
{
	std::shared_ptr<DeviceSpecificFocalPointGain> ptr = std::shared_ptr<DeviceSpecificFocalPointGain>(new DeviceSpecificFocalPointGain());
	ptr->_points = points;
	ptr->_amplitudes = amplitudes;
	ptr->_geometry = autd::GeometryPtr(nullptr);
	return ptr;
}

void autd::DeviceSpecificFocalPointGain::build()
{
	if (this->built()) return;
	if (this->geometry() == nullptr) BOOST_ASSERT_MSG(false, "Geometry is required to build Gain");

	this->_data.clear();
	const int ndevice = this->geometry()->numDevices();
	for (int i = 0; i < ndevice; i++)
	{
		this->_data[this->geometry()->deviceIdForDeviceIdx(i)].resize(NUM_TRANS_IN_UNIT);
	}
	const int ntrans = this->geometry()->numTransducers();
	for (int i = 0; i < ntrans; i++)
	{
		Eigen::Vector3f trp = this->geometry()->position(i);
		Eigen::Vector3f point = this->_points.col(this->geometry()->deviceIdForTransIdx(i));
		float dist = (trp - point).norm();
		float fphase = fmodf(dist, ULTRASOUND_WAVELENGTH) / ULTRASOUND_WAVELENGTH;
		uint8_t phase = round(255.0*(1 - fphase));
		uint8_t amplitude = this->_amplitudes[this->geometry()->deviceIdForTransIdx(i)];
		this->_data[this->geometry()->deviceIdForTransIdx(i)][i%NUM_TRANS_IN_UNIT] = ((uint16_t)amplitude << 8) + phase;
	}
	this->_built = true;
}