#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include "autd3.hpp"
#include "additionalGain.hpp"
#include "StereoTracker.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"

using namespace dynaman;

int main(int argc, char** argv) {

	/*user-defined configurations*/
	const int numTrial = 5;
	const Eigen::Vector3f pos_init(-200, 0, 0);
	const Eigen::Vector3f posCenter(0, 0, 0);
	const float dist_travel = 200;
	const float err_tol = 10;
	const float wait_time_tol = 30000;

	auto stabilized_at_init = [&pos_init, &err_tol](const Eigen::Vector3f& pos) {
		return (pos - pos_init).norm() < err_tol;
	};

	auto cond_finish = [&pos_init](const Eigen::Vector3f& pos, const float dist_travel) {
		std::cout << pos.x() << ", " << pos_init.x() << "," << dist_travel << std::endl;
		return pos.x() >= pos_init.x() + dist_travel;
	};

	std::cout << "Enter prefix:" << std::endl;
	std::string prefix;
	std::cin >> prefix;

	std::string target_image_name("blue_target_no_cover.png");
	/*end of user-defined configurations*/

	auto pObject = dynaman::FloatingObject::Create(
		pos_init,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		-0.036e-3f,
		50.f
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
		5 * Eigen::Vector3f::Constant(-4.0f), // gainD
		1 * Eigen::Vector3f::Constant(-0.05f), //gainI
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
		while (count_wait < 10)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			if (timeGetTime() - timeWaitStart > wait_time_tol) {
				std::cout << "wait time tolerance exceeded." << std::endl;
				break;
			}
			stabilized_at_init(pObject->getPosition()) ? count_wait++ : count_wait = 0;
		}
		Eigen::VectorXi amplitudes(num_device);
		amplitudes << 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0;
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
			auto gain = autd::DeviceSpecificFocalPointGain::Create(pos_object, amplitudes);
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
