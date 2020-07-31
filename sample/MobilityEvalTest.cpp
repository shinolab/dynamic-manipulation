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


	std::string obsLogName("20200730_MobTestObsLog.csv");
	std::string controlLogName("20200730_MobTestControlLog.csv");
	Eigen::Vector3f posStart(-300, -50, 0);
	Eigen::Vector3f posEnd(300, -50, 0);
	int numTrial = 10;

	Eigen::Vector3f pos_init(0, 0, 0);
	std::string target_image_name("blue_target_no_cover.png");
	Eigen::Vector3f pos_sensor(-125.652f, -871.712f, 13.3176f);
	Eigen::Quaternionf quo_sensor(0.695684f, -0.718283f, -0.0089647f, 0.00359883f);
	std::string leftCamId("32434751");
	std::string rightCamId("43435351");
	/*end of user-defined configurations*/

	auto pObject = dynaman::FloatingObject::Create(
		posStart,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		-0.036e-3f,
		50.f
	);

	std::cout << "opening aupa ..." << std::endl;
	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen())
		return ENXIO;
	haptic_icon::SetGeometry(pAupa);
	std::cout << "opening tracker ..." << std::endl;
	auto pTracker = stereoTracker::create(
		stereoCamera::create(ximeaCameraDevice::create(leftCamId), ximeaCameraDevice::create(rightCamId)),
		imgProc::hue_backproject_extractor::create(target_image_name),
		imgProc::hue_backproject_extractor::create(target_image_name),
		pos_sensor,
		quo_sensor
	);
	pTracker->open();

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
	//Recorder recorder;
	//recorder.Start(pObject, filename, 33);
	pManipulator->StartManipulation(pAupa, pTracker, pObject);
	
	for (int iTrial = 0; iTrial < numTrial; iTrial++) {
		std::this_thread::sleep_for(std::chrono::seconds(10));
		pObject->updateStatesTarget(posStart);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posLeft));
		std::this_thread::sleep_for(std::chrono::seconds(10));
		pObject->updateStatesTarget(posEnd);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(3.0f, timeGetTime(), posLeft, posRight));
	}

	pManipulator->FinishManipulation();
	//recorder.Stop();
	pAupa->Close();
	return 0;
}
