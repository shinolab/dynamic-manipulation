#include <memory>
#include <thread>
#include <chrono>
#include "autd3.hpp"
#include "StereoTracker.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"

using namespace dynaman;

int main(int argc, char** argv ) {

	/*user-defined configurations*/
	std::string prefix("20200827_stationary_20cm_daiso_long");
	std::string filename_obs = prefix + "_obs.csv";
	std::string filename_control = prefix + "_control.csv";
	Eigen::Vector3f pos(200, 0, 0);
	//std::string target_image_name("blue_target_no_cover.png");
	std::string target_image_name("blue_20cm_daiso_target.png");
	/*end of user-defined configurations*/

	auto pObject = dynaman::FloatingObject::Create(
		pos,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		-0.036e-3f,
		100.f
	);

	auto pTracker = haptic_icon::CreateTracker(target_image_name);
	pTracker->open();

	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen())
		return ENXIO;
	haptic_icon::SetGeometry(pAupa);

	DWORD timeInit = timeGetTime();
	pAupa->AppendModulationSync(autd::SineModulation::Create(200));
	while (timeGetTime() - timeInit < 10000) {
		Eigen::Vector3f pos;
		DWORD tObserve;
		auto measured = pTracker->observe(tObserve, pos, pObject);
		pAupa->AppendGainSync(autd::FocalPointGain::Create(pos, 255));
		std::this_thread::sleep_for(std::chrono::microseconds(5));
	}
	pAupa->Close();
	return 0;
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
		filename_obs,
		filename_control
	);
	pManipulator->StartManipulation(pAupa, pTracker, pObject);

	std::cout << "press any key to close." << std::endl;
	getchar();

	pManipulator->FinishManipulation();
	pAupa->Close();
	return 0;
}
