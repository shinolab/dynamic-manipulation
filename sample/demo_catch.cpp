#include <memory>
#include <thread>
#include <chrono>
#include "autd3.hpp"
#include "StereoTracker.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"

using namespace dynaman;

int main(int argc, char** argv) {

	/*user-defined configurations*/
	Eigen::Vector3f pos_init(0, 0, 0);
	Eigen::Vector3f pos_left(400, 0, 400);
	Eigen::Vector3f pos_right(-400, 0, 400);
	Eigen::Vector3f pos_left2(200, 0, 200);
	Eigen::Vector3f pos_left_final(400, 100, 400);

	std::string target_image_name("blue_target_r50mm.png");
	/*end of user-defined configurations*/

	auto pObject = dynaman::FloatingObject::Create(
		pos_init,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		0,//-0.036e-3f,
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
		30 * Eigen::Vector3f::Constant(-1.6f), // gainP
		6 * Eigen::Vector3f::Constant(-4.0f), // gainD
		1 * Eigen::Vector3f::Constant(-0.05f), //gainI
		100, //freqLM
		10,
		5,
		0
	);

	pManipulator->StartManipulation(pAupa, pTracker, pObject);

	std::this_thread::sleep_for(std::chrono::seconds(8));

	for (int i = 0; i < 15; i++) {
		//pObject->updateStatesTarget(pos_right);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posLeft));
		//std::this_thread::sleep_for(std::chrono::milliseconds(750));
		//pObject->updateStatesTarget(pos_init);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(3.0f, timeGetTime(), posLeft, posRight));
		//std::this_thread::sleep_for(std::chrono::milliseconds(750));
		pObject->updateStatesTarget(pos_left);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(3.0f, timeGetTime(), posLeft, posRight));
		std::this_thread::sleep_for(std::chrono::milliseconds(750));
		pObject->updateStatesTarget(pos_init);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posRight, posCenter));
		std::this_thread::sleep_for(std::chrono::milliseconds(750));
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	pObject->updateStatesTarget(pos_left_final);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	getchar();
	pManipulator->FinishManipulation();
	pAupa->Close();
	return 0;
}
