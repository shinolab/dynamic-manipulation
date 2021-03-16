#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <ctime>
#include "autd3.hpp"
#include "additionalGain.hpp"
#include "StereoTracker.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"

using namespace dynaman;

int main(int argc, char** argv) {

	/*user-defined configurations*/
	const int numTrial = 1;
	const float err_tol = 10;
	const float wait_time_tol = 20000;
	const int count_wait_max = 10;

	std::string str_radius;
	std::string str_dist;
	std::string str_power;
	std::cout << "Enter radius:" << std::endl;
	std::cin >> str_radius;
	std::cout << "Enter distance:" << std::endl;
	std::cin >> str_dist;
	std::cout << "Enter power (0-1)" << std::endl;
	std::cin >> str_power;

	int radius = std::atoi(str_radius.c_str());
	int dist_travel = std::atoi(str_dist.c_str());
	auto power = std::atof(str_power.c_str());
	auto amplitude = static_cast<int>(255 * std::powf(power, 1.0f / 1.5f));

	std::cout
		<< "radius: " << radius << std::endl
		<< "distance: " << dist_travel << std::endl
		<< "power (amplitude): " << power << "(" << amplitude << ")" << std::endl;

	auto timer = std::time(NULL);
	auto p_time = localtime(&timer);
	char str_time[sizeof("YYYYmmdd_HHMMSS")];
	std::strftime(str_time, sizeof(str_time), "%Y%m%d_%H%M%S", p_time);
	
	std::string prefix = std::string(str_time) + "_r" + str_radius + "_d" + str_dist;
	std::string config_name = prefix + "_config.txt";
	std::ofstream ofs_config(config_name);
	ofs_config << radius << "," << dist_travel << "," << amplitude;
	ofs_config.close();

	const Eigen::Vector3f pos_init(-dist_travel/2, 20, 0);
	const Eigen::Vector3f posCenter(0, 0, 0);

	auto stabilized_at_init = [&pos_init, &err_tol](const Eigen::Vector3f& pos) {
		auto error = (pos - pos_init).norm();
		std::cout << error << std::endl;
		return error < err_tol;
	};
	auto cond_finish = [&pos_init](const Eigen::Vector3f& pos, const float dist_travel) {
		return pos.x() >= pos_init.x() + dist_travel;
	};

	std::string target_image_name("blue_target_no_cover.png");
	/*end of user-defined configurations*/

	auto pObject = dynaman::FloatingObject::Create(
		pos_init,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		-0.036e-3f,
		radius
	);

	auto pTracker = haptic_icon::CreateTracker(target_image_name);
	pTracker->open();

	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen())
		return ENXIO;
	haptic_icon::SetGeometry(pAupa);

	auto pManipulator = MultiplexManipulator::Create(
		20 * Eigen::Vector3f::Constant(-1.6f), // gainP
		2.5 * Eigen::Vector3f::Constant(-4.0f), // gainD
		1 * Eigen::Vector3f::Constant(-0.05f), //gainI
		//20 * Eigen::Vector3f::Constant(-1.6f), // gainP
		//5 * Eigen::Vector3f::Constant(-4.0f), // gainD
		//1 * Eigen::Vector3f::Constant(-0.05f), //gainI
		100, //freqLM
		10,
		5,
		0
	);
	dynamic_cast<MultiplexManipulator*>(pManipulator.get())->EnableLog(
		prefix + "_obs_log.csv",
		prefix + "_control_log.csv"
	);
	pManipulator->StartManipulation(pAupa, pTracker, pObject);
	pObject->updateStatesTarget(posCenter);
	std::this_thread::sleep_for(std::chrono::seconds(2));
	//wait for stabilization
	const int num_device = pAupa->geometry()->numDevices();
	for (int i_trial = 0; i_trial < numTrial; i_trial++) {
		std::cout << "trial #" << i_trial << " has started." << std::endl;
		pObject->updateStatesTarget(pos_init);
		std::this_thread::sleep_for(std::chrono::seconds(3));
		auto timeWaitStart = timeGetTime();
		auto count_wait = 0;
		while (count_wait < count_wait_max)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			if (timeGetTime() - timeWaitStart > wait_time_tol) {
				std::cout << "wait time tolerance exceeded." << std::endl;
				break;
			}
			stabilized_at_init(pObject->getPosition()) ? count_wait++ : count_wait = 0;
		}

		std::atomic<bool> is_finished(false);
		std::string logfilename = prefix + "_" + std::to_string(i_trial) + ".csv";
		auto thr_log = std::thread(
			[&pTracker, &logfilename, &is_finished, &pObject]() 
			{
				std::ofstream ofs_log(logfilename);
				while (!is_finished) {
					DWORD obsTime;
					Eigen::Vector3f pos;
					auto isTracked = pTracker->observe(obsTime, pos, pObject);
					pObject->updateStates(obsTime, pos);
					if (isTracked) {
						ofs_log << obsTime << "," << pos.x() << ", " << pos.y() << ", " << pos.z() << std::endl;
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
				ofs_log.close();
			}
		);
		pManipulator->FinishManipulation();
		auto timeActuateInit = timeGetTime();
		while (timeGetTime() - timeActuateInit < 10000) {
			Eigen::Vector3f pos_object = pObject->getPosition();
			std::map<int, autd::GainPtr> gain_map;
			for (int i_aupa = 0; i_aupa < pAupa->geometry()->numDevices(); i_aupa++) {
				gain_map.insert(
					std::make_pair(
						i_aupa,
						i_aupa == 1 ? autd::FocalPointGain::Create(pos_object, amplitude):autd::NullGain::Create()
					)
				);
			}
			auto gain = autd::GroupedGain::Create(gain_map);
			pAupa->AppendGainSync(gain);
			if (cond_finish(pos_object, dist_travel)) {
				std::cout << "condition satisfied." << std::endl;
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		is_finished = true;
		std::cout << "joining log thread ... " << std::endl;
		thr_log.join();
		std::cout << "restarting control ..." << std::endl;
		pManipulator->StartManipulation(pAupa, pTracker, pObject);
		pObject->updateStatesTarget(posCenter);

	}
	std::cout << "Press any key to close." << std::endl;
	getchar();
	pManipulator->FinishManipulation();
	pAupa->Close();
	return 0;
}
