#include "additionalGain.hpp"
#include "autd3.hpp"
#include <Eigen\Dense>
#include <iostream>

int main() {
	autd::Controller autd;
	autd.Open(autd::LinkType::ETHERCAT);
	if (!autd.isOpen()) {
		return ENXIO;
	}
	autd.geometry()->AddDevice(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0));
	auto foci = Eigen::Matrix3Xf(3, 1);
	foci << 10.16*8.5, 10.16*6.5, 200;
	auto amps = Eigen::VectorXi(1);
	amps << 255;
	autd.AppendGainSync(autd::VortexFocalPointGain::Create(foci, amps, Eigen::VectorXf::Ones(1)));
	//autd.AppendModulationSync(autd::Modulation::Create(255));
	autd.AppendModulationSync(autd::SineModulation::Create(150));
	std::cout << "Press enter to close." << std::endl;
	getchar();
	return 0;
}