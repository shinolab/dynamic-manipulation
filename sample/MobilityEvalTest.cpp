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
	auto direction =  Eigen::Vector3f(1, 0, 1).normalized();
	const float force = 1.5; //[N]

	std::string target_image_name("blue_target_r50mm.png");
	std::string obsLogName("20210723_Bangbang_Obs_xz8_low.csv");
	std::string controlLogName("20210723_Bangbang_Control_xz8_low.csv");
	float dist = 200;
	Eigen::Vector3f posInit(0, 0, 0);
	Eigen::Vector3f posStart = posInit - dist * direction;
	Eigen::Vector3f posEnd = posInit + dist * direction;
	int numTrial = 1;

	//Create Floating Object
	auto pObject = dynaman::FloatingObject::Create(
		posInit,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		0,//-0.036e-3f,
		50.f
	);
	auto traj = TrajectoryBangbangWithDrag::Create(
		force,
		pObject->Radius(),
		timeGetTime(),
		posStart,
		posEnd
	);
	auto time_to_accel = dynamic_cast<TrajectoryBangbangWithDrag*>(traj.get())->time_to_accel();
	auto time_to_decel = dynamic_cast<TrajectoryBangbangWithDrag*>(traj.get())->time_to_decel();
	std::cout << "time to accel: " << time_to_accel << std::endl;
	std::cout << "time to decel: " << time_to_decel << std::endl;

	//return 0;
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
	std::this_thread::sleep_for(std::chrono::seconds(5));// wait for I-gain adjustment`

	pObject->updateStatesTarget(posStart);
	
	std::this_thread::sleep_for(std::chrono::seconds(10)); 

	for (int iTrial = 0; iTrial < numTrial; iTrial++) {
		//pObject->updateStatesTarget(posEnd);
		auto traj = TrajectoryBangbangWithDrag::Create(
			force,
			pObject->Radius(),
			timeGetTime(),
			posStart,
			posEnd
		);
		//std::cout <<dynamic_cast<TrajectoryBangbangWithDrag*>(traj.get())->time_to_accel();

		pObject->SetTrajectory(traj);		
		std::this_thread::sleep_for(std::chrono::seconds(10));
		//pObject->updateStatesTarget(posStart);
		pObject->SetTrajectory(
			TrajectoryBangbangWithDrag::Create(
				force,
				pObject->Radius(),
				timeGetTime(),
				posEnd,
				posStart
			)
		);
		std::this_thread::sleep_for(std::chrono::seconds(10));
	}

	pManipulator->FinishManipulation();

	pAupa->Close();
	return 0;
}
