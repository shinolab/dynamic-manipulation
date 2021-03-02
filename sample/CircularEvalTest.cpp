#include <memory>
#include <thread>
#include <chrono>
#include "autd3.hpp"
#include "StereoTracker.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"
#include "Trajectory.hpp"

using namespace dynaman;

int main(int argc, char** argv) {


	//Log file name
	std::string obsLogName("20210123_circular_obs_r150_1.csv");
	std::string controlLogName("20210123_circular_control_r150_1.csv");

	// params of the circular trajectory
	float orbit_radius = 150;
	float omega = 7.0 * pi / 8.0f;
	float orbit_period = 2.0 * pi / omega;
	float inclination = pi / 2.0f;
	float raan = 0.0f;
	Eigen::Vector3f posCenter(0.0f, 0.0f, 0.0f);
	Eigen::Vector3f posCircleInit = posCenter + orbit_radius * Eigen::Vector3f::UnitX();
	int num_loop_init = 5;
	int num_loop_main = 10;
	int total_time_ms = (num_loop_init + num_loop_main) * orbit_period * 1000;

	std::this_thread::sleep_for(std::chrono::seconds(5));

	/*user-defined configurations*/
	std::string target_image_name("blue_target_no_cover.png");
	/*end of user-defined configurations*/

	auto pObject = dynaman::FloatingObject::Create(
		posCenter,
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
	dynamic_cast<MultiplexManipulator*>(pManipulator.get())->EnableLog(obsLogName, controlLogName);
	pManipulator->StartManipulation(pAupa, pTracker, pObject);

	//traslation maneuver:
	pObject->updateStatesTarget(posCircleInit);
	std::this_thread::sleep_for(std::chrono::seconds(5));

	pObject->SetTrajectory(dynaman::TrajectoryCircle::Create(
		posCenter,
		orbit_radius,
		inclination,
		raan,
		orbit_period,
		0.f,
		timeGetTime())
	);
	std::this_thread::sleep_for(std::chrono::milliseconds(total_time_ms));
	pObject->updateStatesTarget(posCenter);
	std::cout << "Press any key to close." << std::endl;
	getchar();
	pManipulator->FinishManipulation();
	pAupa->Close();
	return 0;
}
