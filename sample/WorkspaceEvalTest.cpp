#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include "autd3.hpp"
#include "StereoTracker.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"
#include "recorder.hpp"

using namespace dynaman;

int main(int argc, char** argv) {

	//configurations
	auto direction = -Eigen::Vector3f::UnitX();
	std::string target_image_name("blue_target_r50mm.png");
	std::string obsLogName("20210405_WorkspaceEval_mx2.csv");
	std::string controlLogName("20210405_WorkspaceEvalControl_mx2.csv");
	float dist_step = 50;
	float dist_max = 450;
	Eigen::Vector3f posInit(0, 0, 0);

	//Create Floating Object
	auto pObject = dynaman::FloatingObject::Create(
		posInit,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		0,
		50.f
	);

	//Create Stereo Tracker	
	std::cout << "opening tracker ..." << std::endl;
	auto pTracker = haptic_icon::CreateTracker(target_image_name);
	pTracker->open();

	std::cout << "opening aupa ..." << std::endl;
	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen())
		return ENXIO;
	haptic_icon::SetGeometry(pAupa);

	auto pManipulator = MultiplexManipulator::Create(
		20 * Eigen::Vector3f::Constant(-1.6f), // gainP
		5 * Eigen::Vector3f::Constant(-4.0f), // gainD
		1 * Eigen::Vector3f::Constant(-0.05f), //gainI
		100 //freqLM
	);
	dynamic_cast<MultiplexManipulator*>(pManipulator.get())->EnableLog(
		obsLogName,
		controlLogName
	);

	pManipulator->StartManipulation(pAupa, pTracker, pObject);

	std::this_thread::sleep_for(std::chrono::seconds(20)); // wait for I-gain adjustment
	for (int i_step = 1; i_step * dist_step < dist_max; i_step++) {
		auto posTgt = static_cast<Eigen::Vector3f>(posInit + i_step * dist_step * direction);
		pObject->updateStatesTarget(posTgt);
		std::cout << "set distance: " << i_step * dist_step << " mm" << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(20));
	}

	pManipulator->FinishManipulation();
	pAupa->Close();
	return 0;

}
