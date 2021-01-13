#include <string>
#include <Eigen/Dense>
#include "StereoTracker.hpp"
#include "autd3.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"
#include "pcl_util.hpp"
#include "pcl_viewer.hpp"
#include "pcl_grabber.hpp"
#include "HandStateReader.hpp"
#include "ActionHandler.hpp"
#include <Windows.h>

#pragma comment(lib, "winmm")

int main(int argc, char** argv) {
	std::string target_image_name("blue_target_no_cover.png");
	float radius = 50;
	float weight = 0;
	auto gainP = 20 * Eigen::Vector3f::Constant(-1.6f);
	auto gainD = 5 * Eigen::Vector3f::Constant(-4.0f);
	auto gainI = Eigen::Vector3f::Constant(-0.05f);
	
	auto pTracker = haptic_icon::CreateTracker(target_image_name);
	pTracker->open();
	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen()) {
		return ENXIO;
	}
	haptic_icon::SetGeometry(pAupa);

	auto pObject = dynaman::FloatingObject::Create(
		Eigen::Vector3f(0, 0, 0),
		Eigen::Vector3f::Constant(-400),
		Eigen::Vector3f::Constant(400),
		weight,
		radius
	);

	auto pManipulator = dynaman::MultiplexManipulator::Create(gainP, gainD, gainI);
	pManipulator->StartManipulation(pAupa, pTracker, pObject);

	
	Eigen::Vector3f pos_rs(-409.233, 460.217, -7.72512);
	Eigen::Matrix3f rot_rs;
	rot_rs <<
		-0.698625, -0.0134938, 0.715361,
		-0.71529, -0.0103563, -0.698751,
		0.0168372, -0.999855, -0.00241686;

	auto grabber = rs2_pcl_grabber::Create(0.001f * pos_rs, rot_rs, "825513025618", 0.15f, 1.0f);
	grabber->Open();
	std::this_thread::sleep_for(std::chrono::seconds(5));
	auto pHandStateReader = dynaman::PclHandStateReader::Create(
		pObject->Radius()
	);
	auto pCloudInit = pHandStateReader->DefaultPreprocess(grabber->Capture(), pObject);
	for (int i = 0; i < 10; i++) {
		if (pHandStateReader->EstimateSphereRadius(pCloudInit, pObject->getPosition())) {
			std::cout << "Estimation succeeded." << std::endl;
			break;
		}
	}
	pcl_viewer viewer("HandStateReaderTest", 1280, 720);
	while (viewer) {
		auto pCloud = pHandStateReader->DefaultPreprocess(grabber->Capture(), pObject);
		dynaman::HandState handState;
		Eigen::Vector3f posBalloon = pObject->getPosition();
		bool read_ok = pHandStateReader->Read(handState, pCloud, posBalloon);
		if (read_ok) {
			switch (handState)
			{
			case dynaman::HandState::NONCONTACT:
				std::cout << "NONCONTACT" << std::endl;
				break;
			case dynaman::HandState::HOLD_FINGER_UP:
				std::cout << "FINGER_UP" << std::endl;
				break;
			case dynaman::HandState::HOLD_FINGER_DOWN:
				std::cout << "FINGER_DOWN" << std::endl;
				break;
			default:
				break;
			}
			auto pCloudBalloon = pcl_util::MakeSphere(posBalloon, pHandStateReader->RadiusObject());
			auto pCloudColliderContact = pcl_util::MakeSphere(posBalloon, pHandStateReader->RadiusColliderContact());
			auto pCloudColliderClick = pcl_util::MakeSphere(posBalloon, pHandStateReader->RadiusColliderClick());
			std::vector<pcl_util::pcl_ptr> cloudPtrs{
				pCloud,
				pCloudBalloon,
				pCloudColliderContact,
				pCloudColliderClick
			};
			viewer.draw(cloudPtrs);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	grabber->Close();
	pManipulator->FinishManipulation();
	return 0;
}