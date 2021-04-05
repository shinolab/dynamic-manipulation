#include <iostream>
#include <fstream>
#include <memory>
#include <thread>
#include <chrono>
#include <random>
#include "autd3.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"
#include "vibrator.hpp"

using namespace dynaman;

int main(int argc, char** argv)
{
	std::string prefix_log;
	std::cout << "Enter the name:";
	std::cin >> prefix_log;
	std::cout << std::endl << "Name: " << prefix_log << std::endl;
	std::cout << "Now initializing ..." << std::endl;
	const float db_step = 1;
	const int num_trial = 3;
	const int tactile_period_ms = 1000;
	std::vector<int> freq_list{200};
	std::vector<float> db_mid_list{ -13 };
	std::vector<float> db_offset_max_list{ 2, 3, 4 };
	std::vector<float> db_offset_min_list{ 4, 5, 6 };
	std::vector<bool> reverse_list{ false, true };
	if (num_trial != db_offset_max_list.size() || num_trial != db_offset_min_list.size()) {
		std::cerr << "error found in configuration parameter." << std::endl;
		return -1;
	}
	if (freq_list.size() != db_mid_list.size()) {
		std::cerr << "error found in freq parameter." << std::endl;
		return -1;
	}
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());

	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen())
		return ENXIO;
	haptic_icon::SetGeometry(pAupa);

	std::string target_image_name("blue_target_r50mm.png");
	auto pTracker = haptic_icon::CreateTracker(target_image_name);

	auto pObject = dynaman::FloatingObject::Create(
		Eigen::Vector3f(0, 0, 0),
		Eigen::Vector3f::Constant(-400),
		Eigen::Vector3f::Constant(400),
		-0.036e-3f,
		50.f
	);
	Vibrator vibrator(prefix_log, pAupa, pTracker);
	vibrator.Start(pObject);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	vibrator.Stop();
	std::ofstream ofs(prefix_log + "_thres.txt", std::ios_base::app);
	for (auto itr_freq = freq_list.begin(); itr_freq != freq_list.end(); itr_freq++) 
	{
		int idx_freq = std::distance(freq_list.begin(), itr_freq);
		auto freq = *itr_freq;
		for (auto&& reversed : reverse_list) {
			vibrator.SetReverse(reversed);
			std::shuffle(db_offset_max_list.begin(), db_offset_max_list.end(), engine);
			std::shuffle(db_offset_min_list.begin(), db_offset_min_list.end(), engine);
			std::cout << "Current frqeuency is: " << freq << " Reverse: " << reversed << std::endl;
			for (int i_trial = 0; i_trial < num_trial; i_trial++) {
				//ascending series
				std::cout << "Ascending Series has started." << std::endl;
				ofs << "A,";
				float db_mid = db_mid_list[idx_freq];
				float db_min = db_mid - db_offset_min_list[i_trial];
				float db_max = db_mid + db_offset_max_list[i_trial];
				for (float db = db_min; db <= db_max ; db += db_step) {
					float amplitude = Vibrator::decibel_to_amp(db);
					std::cout << "current amp is: " << amplitude << " (" << db << "(dB)). press q to abort, or press n to ascend." << std::endl;
					vibrator.SetAmplitude(amplitude);
					vibrator.SetFrequency(freq);
					Beep(440, 500);
					std::this_thread::sleep_for(std::chrono::milliseconds(400));
					vibrator.Start(pObject);
					std::this_thread::sleep_for(std::chrono::milliseconds(tactile_period_ms));
					vibrator.Stop();
					Beep(520, 400);
					std::this_thread::sleep_for(std::chrono::milliseconds(400));
					ofs << db << ",(" << amplitude<< "),";
					bool detected = false;
					while (1) {
						std::string key;
						std::cin >> key;
						if (key == "q") {
							detected = true;
							break;
						}
						if (key == "n") {
							break;
						}
					}
					if (detected) {
						break;
					}
				}
				ofs << std::endl;
				Beep(800, 400);
				while (1) {
					std::cout << "Take a rest. press 'go' to proceed." << std::endl;
					std::string key;
					std::cin >> key;
					if (key == "go") {
						break;
					}
				}
				//descending series
				std::cout << "Descending Series has started." << std::endl;
				ofs << "D,";
				for (float db = db_max; db >= db_min; db -= db_step) {
					float amplitude = Vibrator::decibel_to_amp(db);
					std::cout << "current amp is: " << amplitude << " (" << db << "(dB)). press q to abort, or press n to ascend." << std::endl;
					vibrator.SetAmplitude(amplitude);
					vibrator.SetFrequency(freq);
					Beep(440, 400);
					std::this_thread::sleep_for(std::chrono::milliseconds(400));
					vibrator.Start(pObject);
					std::this_thread::sleep_for(std::chrono::milliseconds(tactile_period_ms));
					vibrator.Stop();
					Beep(520, 400);
					std::this_thread::sleep_for(std::chrono::milliseconds(400));
					ofs << db << ",(" << amplitude << "),";
					bool detected = false;
					while (1) {
						std::string key;
						std::cin >> key;
						if (key == "q") {
							detected = true;
							break;
						}
						if (key == "n") {
							break;
						}
					}
					if (detected) {
						break;
					}
				}
				ofs << std::endl;
				Beep(800, 400);
				while (1) {
					std::cout << "Take a rest. press 'go' to proceed." << std::endl;
					std::string key;
					std::cin >> key;
					if (key == "go") {
						break;
					}
				}
			}
		}
	}
	return 0;
}