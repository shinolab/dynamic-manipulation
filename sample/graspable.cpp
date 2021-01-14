#include <vector>
#include <memory>
#include <iostream>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include "StereoTracker.hpp"
#include "autd3.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"
#include "pcl_viewer.hpp"
#include "pcl_grabber.hpp"
#include "HandStateReader.hpp"
#include "ActionHandler.hpp"
#include "balloon_interface.hpp"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;


int main(int argc, char** argv) {

	std::string target_image_name("blue_target_no_cover.png");
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
		-0.036e-3f,
		50.f
	);

	auto pManipulator = dynaman::MultiplexManipulator::Create(
		20 * Eigen::Vector3f::Constant(-1.6f), // gainP
		5 * Eigen::Vector3f::Constant(-4.0f), // gainD
		1 * Eigen::Vector3f::Constant(-0.05f), //gainI
		100, //freqLM
		10,
		5,
		0
	);
	pManipulator->StartManipulation(pAupa, pTracker, pObject);
	std::this_thread::sleep_for(std::chrono::seconds(5)); // wait until stabilized

	Eigen::Matrix3f rot_rs;
	rot_rs <<
		-0.698625, -0.0134938, 0.715361,
		-0.71529, -0.0103563, -0.698751,
		0.0168372, -0.999855, -0.00241686;
	Eigen::Vector3f pos_rs(-409.233, 460.217, -7.72512);

	auto grabber = rs2_pcl_grabber::Create(0.001f * pos_rs, rot_rs, "825513025618", 0.15f, 1.0f);
	grabber->Open();

	auto pHandStateReader = dynaman::PclHandStateReader::Create(
		0.001f * pObject->Radius()
	);
	auto pCloudInit = pHandStateReader->DefaultPreprocess(grabber->Capture(), pObject);
	for (int i = 0; i < 10; i++) {
		if (pHandStateReader->EstimateSphereRadius(pCloudInit, 0.001f * pObject->getPosition())) {
			std::cout << "Estimation succeeded." << std::endl;
			std::cout << "Balloon Radius: " << pHandStateReader->RadiusObject() << std::endl;
			break;
		}
	}

	auto pActionHandler = dynaman::ActionHandler::create();
	pActionHandler->setOnHold(
		[&pManipulator]() {
			pManipulator->FinishManipulation();
		}
	);
	pActionHandler->setOnRelease(
		[&pManipulator, &pAupa, &pTracker, &pObject]() {
			pManipulator->StartManipulation(pAupa, pTracker, pObject);
		}
	);
	pActionHandler->setAtHeldInit(
		[&pObject]() {
			pObject->updateStatesTarget(pObject->AveragePosition());
		}
	);
	pActionHandler->setAtHeldFingerUp(
		[&pObject]() {
			pObject->updateStatesTarget(pObject->AveragePosition());
		}
	);
	pActionHandler->setAtHeldFingerDown(
		[&pObject]() {
			pObject->updateStatesTarget(pObject->AveragePosition());
		}
	);

	pcl_viewer viewer("pointcloud", 1280, 720);
	while (viewer) {
		auto pCloud = pHandStateReader->DefaultPreprocess(grabber->Capture(), pObject);
		dynaman::HandState handState;
		auto posBalloon = 0.001f * pObject->getPosition();
		bool read_ok = pHandStateReader->Read(handState, pCloud, posBalloon);
		if (read_ok) {
			pActionHandler->update(handState);
		}
		pActionHandler->execute();
		auto pCloudBalloon = pcl_util::MakeSphere(posBalloon, pHandStateReader->RadiusObject());
		auto pCloudColliderContact = pcl_util::MakeSphere(posBalloon, pHandStateReader->RadiusColliderContact());
		auto pCloudInsideColliderClick = pHandStateReader->ExtractPointsInsideColliderClick(pCloud, posBalloon);
		//auto pCloudColliderClick = pcl_util::MakeSphere(0.001f*posBalloon, pHandStateReader->RadiusColliderClick());
		std::vector<pcl_util::pcl_ptr> cloudPtrs{
			//pCloud,
			pCloudBalloon,
			pCloudColliderContact,
			pCloudInsideColliderClick
		};
		viewer.draw(cloudPtrs);

		std::this_thread::sleep_for(std::chrono::microseconds(10));
	}
	grabber->Close();
	pManipulator->FinishManipulation();
	return 0;
}