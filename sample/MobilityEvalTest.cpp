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
	auto direction = Eigen::Vector3f::UnitX();
	std::string target_image_name("blue_target_r50mm.png");
	std::string obsLogName("20210406_MobTestObsLog_x.csv");
	std::string controlLogName("20210406_MobTestControlLog_x.csv");
	float dist = 250;
	Eigen::Vector3f posInit(0, 0, 0);
	Eigen::Vector3f posStart = posInit;- dist * direction;
	Eigen::Vector3f posEnd = posInit + dist * direction;
	int numTrial = 11;

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
	
	std::this_thread::sleep_for(std::chrono::seconds(10)); // wait for I-gain adjustment
	for (int iTrial = 0; iTrial < numTrial; iTrial++) {
		pObject->updateStatesTarget(posStart);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posLeft));
		std::this_thread::sleep_for(std::chrono::seconds(10));
		pObject->updateStatesTarget(posEnd);//pObject->SetTrajectory(dynaman::TrajectoryBangBang::Create(3.0f, timeGetTime(), posLeft, posRight));
		std::this_thread::sleep_for(std::chrono::seconds(10));
	}

	pManipulator->FinishManipulation();
	//recorder.Stop();
	pAupa->Close();
	return 0;
}
