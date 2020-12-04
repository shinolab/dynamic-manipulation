#include <iostream>
#include <fstream>
#include <memory>
#include <thread>
#include <chrono>
#include <random>
#include "autd3.hpp"
#include "StereoTracker.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"
#include "additionalGain.hpp"

class Vibrator{
private:
	std::shared_ptr<autd::Controller> m_aupa;
	std::shared_ptr<dynaman::Tracker> m_tracker;
	std::mutex m_mtxamp;
	std::mutex m_mtxRunning;
	std::thread m_thrVib;
	float m_amp = 0;
	int m_freq = 0;
	bool m_isRunning = false;
	bool m_reversed = false;
	std::string m_filename;
	
public:
	Vibrator(std::string filename):m_filename(filename) {
		m_aupa = std::make_shared<autd::Controller>();
		m_aupa->Open(autd::LinkType::ETHERCAT);
		haptic_icon::SetGeometry(m_aupa);
		std::string target_image_name("blue_target_no_cover.png");
		m_tracker = haptic_icon::CreateTracker(target_image_name);
		m_tracker->open();
	}
	~Vibrator() {
		m_aupa->Close();
	}

	void Start(dynaman::FloatingObjectPtr pObject){
		m_thrVib = std::thread([this, &pObject]() 
			{
				std::ofstream ofs(m_filename + "_pos.csv", std::ios_base::app);
				{
					std::lock_guard<std::mutex> lock(m_mtxRunning);
					m_isRunning = true;
				}
				//m_aupa->AppendModulationSync(autd::SineModulation::Create(200));
				while (m_isRunning) {
					DWORD tObserve;
					Eigen::Vector3f pos;
					bool observed = m_tracker->observe(tObserve, pos, pObject);
					if (observed) {
						std::lock_guard<std::mutex> lock(m_mtxamp);
						autd::GainPtr gain;
						if (m_reversed) {
							gain = autd::ReversibleFocalPointGain::Create(pos, 255, [](const Eigen::Vector3f& pos)
								{
									return pos.x() > 0 ? false : true;
								}
							);
						}
						else {
							gain = autd::FocalPointGain::Create(pos, 255);
						}
						m_aupa->AppendGainSync(gain);
						m_aupa->AppendModulationSync(autd::SineModulation::Create(m_freq, m_amp, 0.5f * m_amp));
						ofs << tObserve << "," << pos.x() << "," << pos.y() << "," << pos.z() << std::endl;
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(30));
				}
				ofs.close();
				m_aupa->AppendGainSync(autd::NullGain::Create());
			}
		);
	}

	void Stop() {
		{
			std::lock_guard<std::mutex> lock(m_mtxRunning);
			m_isRunning = false;
		}
		if (m_thrVib.joinable()) {
			m_thrVib.join();
		}
	}

	void SetAmplitude(float amplitude) {
		std::lock_guard<std::mutex> lock(m_mtxamp);
		m_amp = amplitude;
	}

	void SetFrequency(int freq) {
		std::lock_guard<std::mutex> lock(m_mtxamp);
		m_freq = freq;
	}

	void SetReverse(bool reverse) {
		std::lock_guard<std::mutex> lock(m_mtxamp);
		m_reversed = reverse;
	}

};

float decibel_to_amp(float db) {
	return pow(10, db / 15);
}

float amp_to_decibel(float amplitude) {
	return 15 * log10f(amplitude);
}

int main(int argc, char** argv) {

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
	std::vector<float> db_offset_max_list{ 6, 7, 8 };
	std::vector<float> db_offset_min_list{ 5, 6, 7 };
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

	
	auto pObject = dynaman::FloatingObject::Create(
		Eigen::Vector3f(0, 0, 0),
		Eigen::Vector3f::Constant(-400),
		Eigen::Vector3f::Constant(400),
		-0.036e-3f,
		50.f
	);
	Vibrator vibrator(prefix_log);
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
					float amplitude = decibel_to_amp(db);
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
					float amplitude = decibel_to_amp(db);
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