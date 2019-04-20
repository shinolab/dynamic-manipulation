#include <Eigen\Dense>
#include <boost\assert.hpp>
#include "autd3.hpp"
#include "additionalGain.hpp"
#include <iostream>

autd::GainPtr autd::DeviceSpecificFocalPointGain::Create(Eigen::MatrixXf const &points, Eigen::VectorXi const &amplitudes)
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

autd::GainPtr autd::GaussianBeamGain::Create(Eigen::Vector3f const &point, Eigen::Vector3f const &direction, int const &amplitude, float const &beamAngle)
{
	std::shared_ptr<GaussianBeamGain> ptr = std::shared_ptr<GaussianBeamGain>(new GaussianBeamGain());
	ptr->_point = point;
	ptr->_direction = direction.normalized();
	ptr->_amplitude = amplitude;
	ptr->_cosBeamAngle = cos(beamAngle);
	ptr->_geometry = autd::GeometryPtr(nullptr);
	return ptr;
}

void autd::GaussianBeamGain::build() 
{
	int count = 0;
	if (this->built()) return;
	if (this->geometry() == nullptr) BOOST_ASSERT_MSG(false, "Geometry is required to build Gain.");
	this->_data.clear();
	const int nDevice = geometry()->numDevices();
	for (int i = 0; i < nDevice; i++) {
		this->_data[this->geometry()->deviceIdForDeviceIdx(i)].resize(NUM_TRANS_IN_UNIT);
	}
	const int nTrans = this->geometry()->numTransducers();
	for (int i = 0; i < nTrans; i++){
		Eigen::Vector3f trp = this->geometry()->position(i);
		Eigen::Vector3f dr = this->_point - trp;
		float cosAngle =this->_direction.dot(dr.normalized());
		if (_cosBeamAngle <= cosAngle){
			count++;
			float dist = dr.norm();
			float fphase = fmodf(dist, ULTRASOUND_WAVELENGTH) / ULTRASOUND_WAVELENGTH;
			uint8_t amp = _amplitude;
			uint8_t phase = round(255.0*(1 - fphase));
			this->_data[this->geometry()->deviceIdForTransIdx(i)][i%NUM_TRANS_IN_UNIT] = ((uint16_t)amp << 8) + phase;
		}
		else {
			uint8_t amp = 0x00;
			uint8_t phase = 0x00;
			this->_data[this->geometry()->deviceIdForTransIdx(i)][i%NUM_TRANS_IN_UNIT] = ((uint16_t)amp << 8) + phase;
		}
	}
	std::cout << count << " transducers operating." << std::endl;
}