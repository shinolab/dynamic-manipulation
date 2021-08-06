#include <vector>
#include <memory>
#include <iostream>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/highgui.hpp>
#include "DrawUtil.hpp"
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

namespace {
	const int font_size = 7;
	const int line_width = 8;
	cv::Size screen_size(1920, 1080);
	cv::Scalar touch_color(0, 0, 0);
	cv::Scalar grasp_color(0, 155, 0);
	cv::Scalar click_color(0, 0, 255);
	cv::Scalar release_color(255, 0, 0);
}

void displayText(const std::string& text, cv::Scalar color, bool is_instant = true) {
	
	cv::Mat image(screen_size, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::putText(image, text, cv::Point(200, 540), cv::FONT_HERSHEY_SIMPLEX, font_size, color, line_width);
	cv::imshow("hand_state", image);
	cv::waitKey(5);
	if (is_instant) {
		cv::Mat image_white(screen_size, CV_8UC3, cv::Scalar(255, 255, 255));
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		cv::imshow("hand_state", image_white);
		cv::waitKey(5);

	}
};

int main(int argc, char** argv) {

	Eigen::Vector3f sensor_bias(30, 10, 0);
	Eigen::Vector3f pos_init(0, 0, 0);
	std::string target_image_name("blue_target_r50mm.png");
	auto pTracker = haptic_icon::CreateTracker(target_image_name);
	pTracker->open();
	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen()) {
		return ENXIO;
	}
	haptic_icon::SetGeometry(pAupa);

	auto pObject = dynaman::FloatingObject::Create(
		pos_init,
		Eigen::Vector3f::Constant(-250),
		Eigen::Vector3f::Constant(250),
		0,
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
		if (pHandStateReader->EstimateSphereRadius(pCloudInit, 0.001f * (pObject->getPosition() + sensor_bias))) {
			std::cout << "Estimation succeeded." << std::endl;
			std::cout << "Balloon Radius: " << pHandStateReader->RadiusObject() << std::endl;
			break;
		}
	}

	auto pActionHandler = dynaman::ActionHandler::create();
	pActionHandler->setOnHold(
		[&pManipulator]() {
			std::cout << "Translate to HOLD state" << std::endl;
		}
	);
	pActionHandler->setOnTouch(
		[]() {
			displayText("Touch", touch_color);
		}
	);
	pActionHandler->setOnRelease(
		[&pManipulator]() {
			//pManipulator->ResumeManipulation();
			std::cout << "Translate to RELEASE state" << std::endl;
			displayText("Release", release_color);
		}
	);
	pActionHandler->setOnClick(
		[&pActionHandler, &pObject, &pos_init]() {
			std::cout << "CLICK" << std::endl;
			displayText("Click", click_color);
		}
	);
	pActionHandler->setAtHeldInit(
		[&pObject]() {
			pObject->updateStatesTarget(pObject->getPosition());
			//pObject->resetIntegral();
			displayText("Grasp", grasp_color, false);

		}
	);
	pActionHandler->setAtHeldFingerUp(
		[&pObject]() {
			pObject->updateStatesTarget(pObject->getPosition());
			displayText("Grasp", grasp_color, false);
			std::cout << "Finger UP" << std::endl;
			//pObject->resetIntegral();
		}
	);
	pActionHandler->setAtHeldFingerDown(
		[&pObject]() {
			pObject->updateStatesTarget(pObject->getPosition());
			displayText("Grasp", grasp_color, false);
			std::cout << "Finger DOWN" << std::endl;
			//pObject->resetIntegral();
		}
	);

	//pcl_viewer viewer("pointcloud", 1280, 720);
	cv::Mat image(screen_size, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::putText(image, "Start", cv::Point(200, 540), cv::FONT_HERSHEY_SIMPLEX, font_size, cv::Scalar::all(0), line_width);
	DrawUtil::imshowPopUp("hand_state", image, 1920, 0);
	cv::waitKey(5);
	auto time_init = timeGetTime();
	while (timeGetTime() - time_init < 30000){
		auto pCloud = pHandStateReader->DefaultPreprocess(grabber->Capture(), pObject);
		dynaman::HandState handState = dynaman::HandState::NONCONTACT;
		auto posBalloon = 0.001f * (pObject->getPosition() + sensor_bias);
		bool read_ok = pHandStateReader->Read(handState, pCloud, posBalloon);
		pActionHandler->update(handState);
		pActionHandler->execute();

		if (read_ok) {
			auto pCloudBalloon = pcl_util::MakeSphere(posBalloon, pHandStateReader->RadiusObject());
			auto pCloudColliderContact = pcl_util::MakeSphere(posBalloon, pHandStateReader->RadiusColliderContact());
			auto pCloudInsideColliderClick = pHandStateReader->ExtractPointsInsideColliderClick(pCloud, posBalloon);
			//auto pCloudColliderClick = pcl_util::MakeSphere(0.001f*posBalloon, pHandStateReader->RadiusColliderClick());
			std::vector<pcl_util::pcl_ptr> cloudPtrs{
				pCloud,
				pCloudBalloon
				//pCloudColliderContact,
				//pCloudInsideColliderClick
			};
			//viewer.draw(cloudPtrs);
		}
		std::this_thread::sleep_for(std::chrono::microseconds(1));
	}
	grabber->Close();
	pManipulator->FinishManipulation();
	return 0;
}