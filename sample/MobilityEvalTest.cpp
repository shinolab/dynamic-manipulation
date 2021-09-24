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
#include "GainPlan.hpp"

using namespace dynaman;

float MuxMaximumThrust(
	const Eigen::Vector3f& posStart,
	const Eigen::Vector3f& posEnd,
	float duty_max,
	int num_points,
	autd::GeometryPtr geo,
	std::shared_ptr<arfModelLinearBase> arf_model
) {
	std::vector<MuxThrustSearcher> searchers{
		{geo, arf_model, 1, duty_max},
		{geo, arf_model, 2, duty_max},
		{geo, arf_model, 3, duty_max},
		{geo, arf_model, 4, duty_max}
	};
	float dist = (posEnd - posStart).norm();
	auto direction = (posEnd - posStart).normalized();
	float interval = dist / (num_points - 1);
	float force_min = FLT_MAX;
	for (int i_point = 0; i_point < num_points; i_point++) {
		Eigen::Vector3f pos = posStart + i_point * interval * direction;
		Eigen::MatrixXf posRel = pos.replicate(1, geo->numDevices()) - CentersAutd(geo);
		Eigen::MatrixXf arfMat = arf_model->arf(posRel, RotsAutd(geo));
		float force_max_at_point = 0;
		std::for_each(searchers.begin(), searchers.end(), [&](MuxThrustSearcher& searcher)
			{
				auto duty = searcher.Search(pos, direction);
				auto force = (arfMat * duty).dot(direction);
				if (force > force_max_at_point) {
					force_max_at_point = force;
				}
			}
		);
		if (force_max_at_point < force_min) {
			force_min = force_max_at_point;
		}
	}
	return force_min;
}

int main(int argc, char** argv) {

	//configurations

	auto direction = Eigen::Vector3f(-1, 0, 0).normalized();
	const float duty_max = 0.5;
	const float num_search_points = 10;


	std::string target_image_name("blue_target_r50mm.png");
	std::string obsLogName("20210922_Bangbang_demo.csv");
	std::string controlLogName("20210922_Bangbang_Control_demo.csv");
	float dist = 200;
	Eigen::Vector3f posInit(0, 0, 0);
	Eigen::Vector3f posStart = posInit - dist * direction;
	Eigen::Vector3f posEnd = posInit + dist * direction;
	const int numTrial = 10;

	//Create Floating Object
	auto pObject = dynaman::FloatingObject::Create(
		posInit,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		0,//-0.036e-3f,
		50.f
	);


	std::cout << "opening aupa ..." << std::endl;
	auto pAupa = std::make_shared<autd::Controller>();

	haptic_icon::SetGeometry(pAupa);

	auto arf_model = std::make_shared<arfModelFocusSphereExp50mm>();

	float mu = 3 * 0.4f / 8.0f / pObject->Radius();
	float dist_sw = 0.5f / mu * logf(0.5f * (expf(2.f * mu * 2.0f * dist) + 1.0f));

	Eigen::Vector3f pos_sw_fw = posStart + dist_sw * direction;
	Eigen::Vector3f pos_sw_bw = posEnd - dist_sw * direction;
	float force_accel_fw = MuxMaximumThrust(posStart, pos_sw_fw, duty_max, num_search_points, pAupa->geometry(), arf_model);
	float force_decel_fw = MuxMaximumThrust(posEnd, pos_sw_fw, duty_max, num_search_points, pAupa->geometry(), arf_model);

	float force_accel_bw = MuxMaximumThrust(posStart, pos_sw_bw, duty_max, num_search_points, pAupa->geometry(), arf_model);
	float force_decel_bw = MuxMaximumThrust(posEnd, pos_sw_bw, duty_max, num_search_points, pAupa->geometry(), arf_model);

	std::cout << "force_accel_fw: " << force_accel_fw << std::endl;
	std::cout << "force_decel_fw: " << force_decel_fw << std::endl;


	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen())
		return ENXIO;

	//Create Stereo Tracker	
	std::cout << "opening tracker ..." << std::endl;
	auto pTracker = haptic_icon::CreateTracker(target_image_name);
	pTracker->open();

	auto pManipulator = MultiplexManipulator::Create(
		20 * Eigen::Vector3f::Constant(-1.6f), // gainP
		5 * Eigen::Vector3f::Constant(-4.0f), // gainD
		1 * Eigen::Vector3f::Constant(-0.05f), //gainI
		125, //freqLM
		8,
		5,
		0.0f,
		arf_model
	);
	dynamic_cast<MultiplexManipulator*>(pManipulator.get())->EnableLog(
		obsLogName,
		controlLogName
	);

	auto traj_forward = TrajectoryBangbangWithDrag::Create(
		std::min(force_accel_fw, force_decel_fw),
		pObject->Radius(),
		timeGetTime(),
		posStart,
		posEnd
	);


	auto time_to_accel_fw = dynamic_cast<TrajectoryBangbangWithDrag*>(traj_forward.get())->time_to_accel();
	auto time_to_decel_fw = dynamic_cast<TrajectoryBangbangWithDrag*>(traj_forward.get())->time_to_decel();
	std::cout << "time to accel: " << time_to_accel_fw << std::endl;
	std::cout << "time to decel: " << time_to_decel_fw << std::endl;

	pManipulator->StartManipulation(pAupa, pTracker, pObject);
	std::this_thread::sleep_for(std::chrono::seconds(10));// wait for I-gain adjustment`
	std::cout << "posStart:" << posStart.transpose() << std::endl;
	std::cout << "posEnd:" << posEnd.transpose() << std::endl;
	std::cout << "step-responce" << std::endl;
	pObject->updateStatesTarget(posStart);
	std::this_thread::sleep_for(std::chrono::seconds(3));
	//pObject->updateStatesTarget(posEnd);
	//std::this_thread::sleep_for(std::chrono::seconds(3));
	//pObject->updateStatesTarget(posStart);
	//std::this_thread::sleep_for(std::chrono::seconds(3));
	for (int iTrial = 0; iTrial < numTrial; iTrial++) {
		std::cout << "Moving to the starting point ..." << std::endl;
		//pObject->updateStatesTarget(posStart);
		//std::this_thread::sleep_for(std::chrono::seconds(10));
		//pObject->updateStatesTarget(posEnd);
		float umax = 0.6; +0.1 * iTrial;
		std::cout << "umax: " << umax << std::endl;
		//float force_accel_fw = MuxMaximumThrust(posStart, pos_sw_fw, umax, num_search_points, pAupa->geometry(), arf_model);
		//float force_decel_fw = MuxMaximumThrust(posEnd, pos_sw_fw, umax, num_search_points, pAupa->geometry(), arf_model);

		pObject->SetTrajectory(
			TrajectoryBangbangWithDrag::Create(
				std::min(force_accel_fw, force_decel_fw),
				pObject->Radius(),
				timeGetTime(),
				posStart,
				posEnd
			)
		);
		std::this_thread::sleep_for(std::chrono::milliseconds(2500));
		//pObject->updateStatesTarget(posStart);

		pObject->SetTrajectory(
			TrajectoryBangbangWithDrag::Create(
				std::min(force_accel_bw, force_decel_bw),
				pObject->Radius(),
				timeGetTime(),
				posEnd,
				posStart
			)
		);
		std::this_thread::sleep_for(std::chrono::milliseconds(2500));
	}


	pManipulator->FinishManipulation();

	pAupa->Close();
	return 0;
}
