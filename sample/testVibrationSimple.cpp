#include <iostream>
#include <fstream>
#include <memory>
#include <thread>
#include <chrono>
#include "autd3.hpp"

int main(int argc, char** argv) {
	Eigen::Vector3f origin(10.16* 8.5, 10.16 * 6.5, 341.f);
	float height = 108;
	Eigen::Vector3f focalpoint = origin - Eigen::Vector3f(0, 0, height);

	std::string prefix_log;
	std::cout << "Enter the name:";
	std::cin >> prefix_log;
	std::cout << std::endl << "Name: " << prefix_log << std::endl;

	const int amp_min = 1;
	const int amp_max = 255;
	const int amp_delta = 2;
	const int num_trial = 3;
	std::vector<int> freq_list{ 200 };

	autd::Controller aupa;
	aupa.Open(autd::LinkType::ETHERCAT);
	if (!aupa.isOpen()) {
		return ENXIO;
	}
	aupa.geometry()->AddDevice(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());

	std::ofstream ofs(prefix_log + "thres.txt");
	for (auto&& freq : freq_list) {
		std::cout << "Current frqeuency is: " << freq << std::endl;
		aupa.AppendModulationSync(autd::SineModulation::Create(freq));
		for (int i = 0; i < num_trial; i++) {
			//ascending series

			std::cout << "Ascending Series has started." << std::endl;
			ofs << "A,";
			for (float amp_output = amp_min; amp_output <= amp_max; ) {
				std::string key;
				std::cout << "current amplitude is: " << amp_output << ". press q to abort, or press enter to ascend." << std::endl;
				aupa.AppendGainSync(autd::FocalPointGain::Create(focalpoint, amp_output));

				ofs << amp_output << ",";
				std::cin >> key;
				if (key == "q") {
					break;
				}
				if (key == "n") {
					amp_output += amp_delta;
				}
			}
			ofs << std::endl;

			//descending series
			std::cout << "Descending Series has started." << std::endl;
			ofs << "D,";
			for (float amp_output = amp_max; amp_output >= amp_min; ) {
				std::string key;
				std::cout << "current amplitude is: " << amp_output << ". press q to abort, or press n to descend." << std::endl;
				aupa.AppendGainSync(autd::FocalPointGain::Create(focalpoint, amp_output));
				aupa.AppendModulationSync(autd::SineModulation::Create(freq));
				ofs << amp_output << ",";
				std::cin >> key;
				if (key == "q") {
					break;
				}
				if (key == "n") {
					amp_output -= amp_delta;
				}
			}
			ofs << std::endl;
		}
	}
	aupa.Close();
	return 0;
}