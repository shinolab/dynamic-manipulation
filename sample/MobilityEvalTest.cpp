#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include "autd3.hpp"
#include "StereoTracker.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"
#include "recorder.hpp"
#include "ThrustSearch.hpp"

using namespace dynaman;

int main(int argc, char** argv) {

	//configurations
	auto direction =  Eigen::Vector3f(1, 0, 1).normalized();
	const float force = 1.5; //[mN]
	const float duty_max = 0.6;
	const float num_search_points = 10;

	std::string target_image_name("blue_target_r50mm.png");
	std::string obsLogName("20210819_Pid_Obs_xz5.csv");
	std::string controlLogName("20210819_Pid_Control_xz5.csv");
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

	auto arf_model = std::make_shared<arfModelFocusSphereExp50mm>();
	auto pManipulator = MultiplexManipulator::Create(
		20 * Eigen::Vector3f::Constant(-1.6f), // gainP
		5 * Eigen::Vector3f::Constant(-4.0f), // gainD
		1 * Eigen::Vector3f::Constant(-0.05f), //gainI
		170, //freqLM
		6,
		5,
		0.0f,
		arf_model
	);
	dynamic_cast<MultiplexManipulator*>(pManipulator.get())->EnableLog(
		obsLogName,
		controlLogName
	);

	float mu = 3 * 0.4f / 8.0f / pObject->Radius();
	float dist_sw = 0.5f / mu * logf(0.5f * (expf(2.f * mu * 2.0f * dist) + 1.0f));
	std::cout << "dist_sw: " << dist_sw << std::endl;
	Eigen::Vector3f pos_sw_fw = posStart + dist_sw * direction;
	Eigen::Vector3f pos_sw_bw = posEnd - dist_sw * direction;
	float force_accel_fw = MaximumThrust(posStart, posEnd, duty_max, num_search_points, pAupa->geometry(), arf_model);
	float force_decel_fw = MaximumThrust(posEnd, pos_sw_fw, duty_max, num_search_points, pAupa->geometry(), arf_model);
	float force_accel_bw = MaximumThrust(posEnd, pos_sw_bw, duty_max, num_search_points, pAupa->geometry(), arf_model);
	float force_decel_bw = MaximumThrust(posStart, pos_sw_bw, duty_max, num_search_points, pAupa->geometry(), arf_model);
	std::cout << "force_accel_fw: " << force_accel_fw << std::endl;
	std::cout << "force_decel_fw: " << force_decel_fw << std::endl;
	std::cout << "force_accel_bw: " << force_accel_bw << std::endl;
	std::cout << "force_decel_bw: " << force_decel_bw << std::endl;


	auto traj_forward = TrajectoryBangbangWithDrag::Create(
		std::min(force_accel_fw, force_decel_fw),
		pObject->Radius(),
		timeGetTime(),
		posStart,
		posEnd
	);

	auto traj_backward = TrajectoryBangbangWithDrag::Create(
		std::min(force_accel_bw, force_decel_bw),
		pObject->Radius(),
		timeGetTime(),
		posEnd,
		posStart
	);

	auto time_to_accel_fw = dynamic_cast<TrajectoryBangbangWithDrag*>(traj_forward.get())->time_to_accel();
	auto time_to_decel_fw = dynamic_cast<TrajectoryBangbangWithDrag*>(traj_forward.get())->time_to_decel();
	std::cout << "time to accel: " << time_to_accel_fw << std::endl;
	std::cout << "time to decel: " << time_to_decel_fw << std::endl;
	auto time_to_accel_bw = dynamic_cast<TrajectoryBangbangWithDrag*>(traj_backward.get())->time_to_accel();
	auto time_to_decel_bw = dynamic_cast<TrajectoryBangbangWithDrag*>(traj_backward.get())->time_to_decel();
	std::cout << "time to accel: " << time_to_accel_bw << std::endl;
	std::cout << "time to decel: " << time_to_decel_bw << std::endl;

	pManipulator->StartManipulation(pAupa, pTracker, pObject);
	std::this_thread::sleep_for(std::chrono::seconds(5));// wait for I-gain adjustment`

	pObject->updateStatesTarget(posStart);
	
	std::this_thread::sleep_for(std::chrono::seconds(10)); 

	for (int iTrial = 0; iTrial < numTrial; iTrial++) {
		pObject->updateStatesTarget(posEnd);
		//pObject->SetTrajectory(
		//	TrajectoryBangbangWithDrag::Create(
		//		std::min(force_accel_fw, force_decel_fw),
		//		pObject->Radius(),
		//		timeGetTime(),
		//		posStart,
		//		posEnd
		//	)
		//);		
		std::this_thread::sleep_for(std::chrono::seconds(10));

		pObject->updateStatesTarget(posStart);
		//pObject->SetTrajectory(
		//	TrajectoryBangbangWithDrag::Create(
		//		std::min(force_accel_bw, force_decel_bw),
		//		pObject->Radius(),
		//		timeGetTime(),
		//		posEnd,
		//		posStart
		//	)
		//);
		std::this_thread::sleep_for(std::chrono::seconds(10));
	}

	pManipulator->FinishManipulation();

	pAupa->Close();
	return 0;
}
