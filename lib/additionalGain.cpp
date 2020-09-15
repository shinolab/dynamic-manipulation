#include <Eigen\Dense>
#include <boost\assert.hpp>
#include "autd3.hpp"
#include "additionalGain.hpp"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

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
}

autd::GainPtr autd::VortexFocalPointGain::Create(Eigen::Matrix3Xf const &points,
	Eigen::VectorXi const &amplitudes,
	Eigen::VectorXf const &chilarities) {
	auto ptr = std::shared_ptr<VortexFocalPointGain>(new VortexFocalPointGain());
	ptr->_points = points;
	ptr->_amplitudes = amplitudes;
	ptr->_chiralities = chilarities;
	ptr->_geometry = autd::GeometryPtr(nullptr);
	return ptr;
}

void autd::VortexFocalPointGain::build() {
	int count = 0;
	if (this->built()) return;
	if (this->geometry() == nullptr) BOOST_ASSERT_MSG(false, "Geometry is required to build Gain.");
	
	this->_data.clear();
	const int ndevice = this->geometry()->numDevices();
	for (int idevice = 0; idevice < ndevice; idevice++)
	{
		Eigen::Vector3f center = 0.5 * (this->geometry()->position(idevice*NUM_TRANS_IN_UNIT)
			+ this->geometry()->position((idevice + 1)*NUM_TRANS_IN_UNIT - 1));
		Eigen::Vector3f xaxis = ((this->geometry()->position(idevice*NUM_TRANS_IN_UNIT + 1) 
			- this->geometry()->position(idevice*NUM_TRANS_IN_UNIT))).normalized();
		Eigen::Vector3f yaxis = ((this->geometry()->position(idevice*NUM_TRANS_IN_UNIT + 18)
			- this->geometry()->position(idevice*NUM_TRANS_IN_UNIT))).normalized();
		uint8_t amplitude = this->_amplitudes[idevice];
		std::cout << "center: " << center.x() << ", " << center.y() << ", " << center.z() << std::endl;
		std::cout << "xaxis: " << xaxis.x() << ", " << xaxis.y() << ", " << xaxis.z() << std::endl;
		std::cout << "yaxis: " << yaxis.x() << ", " << yaxis.y() << ", " << yaxis.z() << std::endl;
		this->_data[this->geometry()->deviceIdForDeviceIdx(idevice)].resize(NUM_TRANS_IN_UNIT);
		for (int itrans = 0; itrans < NUM_TRANS_IN_UNIT; itrans++) {
			Eigen::Vector3f trp = this->geometry()->position(idevice * NUM_TRANS_IN_UNIT + itrans);
			Eigen::Vector3f point = this->_points.col(idevice);
			Eigen::Vector3f trp_local = trp - center;
			float dist = (trp - point).norm();
		
			/*
			float fphase = fmodf(dist, ULTRASOUND_WAVELENGTH) / ULTRASOUND_WAVELENGTH
				- _chiralities[idevice] * atan2f(yaxis.dot(trp_local), xaxis.dot(trp_local));

			*/
			float fphase = _chiralities[idevice] * (atan2f(yaxis.dot(trp_local), xaxis.dot(trp_local)) + M_PI) / 2.f / M_PI;

			uint8_t phase = round(255.0*(1 - fphase));
			this->_data[idevice][itrans] = ((uint16_t)amplitude << 8) + phase;
			std::cout << fphase << ",";
		}
	}
}

autd::GainPtr autd::ReversibleFocalPointGain::Create(
	const Vector3f& point, 
	uint8_t amp,  
	std::function<bool(const Eigen::Vector3f&)> condReverse
) {
	auto gain = CreateHelper<ReversibleFocalPointGain>();
	gain->_point = point;
	gain->_geometry = nullptr;
	gain->_amp = amp;
	gain->_condReverse = condReverse;
	return gain;
}

void autd::ReversibleFocalPointGain::build() {
	if (this->built()) return;

	auto geo = this->geometry();
	if (geo == nullptr) BOOST_ASSERT_MSG(false, "Geometry is required to build Gain");

	this->_data.clear();

	const auto ndevice = geo->numDevices();
	for (int i = 0; i < ndevice; i++) {
		this->_data[geo->deviceIdForDeviceIdx(i)].resize(NUM_TRANS_IN_UNIT);
	}

	const auto ntrans = geo->numTransducers();
	for (int i = 0; i < ntrans; i++) {
		const auto trp = geo->position(i);
		const auto dist = (trp - this->_point).norm();
		const auto fphase = fmodf(dist, ULTRASOUND_WAVELENGTH) / ULTRASOUND_WAVELENGTH;
		const auto phase = static_cast<uint8_t>(PhaseShift(trp, round(255.0f * (1.0f - fphase))));
		uint8_t D, S;
		SignalDesign(this->_amp, phase, D, S);
		this->_data[geo->deviceIdForTransIdx(i)].at(i % NUM_TRANS_IN_UNIT) = (static_cast<uint16_t>(D) << 8) + S;
	}

	this->_built = true;
}

double autd::ReversibleFocalPointGain::PhaseShift(const Eigen::Vector3f& pos, double phase) {
	return _condReverse(pos) ? (phase > 0.5f ? phase - 0.5f : phase + 0.5f) : phase;
}