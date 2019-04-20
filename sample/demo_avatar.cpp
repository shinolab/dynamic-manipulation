#include "init_uist.hpp"
#include "odcs.hpp"
#include "projector.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <chrono>
#include <thread>
#include <iostream>

#include <Windows.h>

#pragma comment(lib, "winmm")

int main() {

	float translationTime = 1.0f;
	float transDist = 100.f;

	std::string projectorName = "projector1";
	projector proj(projectorName);
	proj.CreateScreen();
	cv::VideoCapture cap0(0);
	if (!cap0.isOpened()) {
		std::cerr << "ERROR: failed to open camera." << std::endl;
		return -1;
	}
	cv::VideoCapture cap1(1);
	if (!cap1.isOpened()) {
		std::cerr << "ERROR: failed to open camera." << std::endl;
		return -1;
	}

	

	odcs dynaman;
	initialize_uist_setup(dynaman);

	FloatingObjectPtr objPtr1 = FloatingObject::Create(Eigen::Vector3f(436.14, 596.04, 1331.f), -0.0001f);
	FloatingObjectPtr objPtr2 = FloatingObject::Create(Eigen::Vector3f(-423.86f, 596.04f, 1331.f), -0.0001f);

	dynaman.RegisterObject(objPtr1);
	dynaman.RegisterObject(objPtr2);
	dynaman.StartControl();

	while (1) {
		if (true)
		{
			cv::Mat face0;
			cap0.read(face0);
			cv::Mat mask0 = cv::Mat::zeros(face0.rows, face0.cols, CV_8UC1);
			cv::circle(mask0, cv::Point(face0.cols / 2, face0.rows / 2), face0.cols / 4, cv::Scalar::all(255), -1);
			cv::Mat face0_masked;
			face0.copyTo(face0_masked, mask0);
			cv::Mat face1;
			cap1.read(face1);
			cv::Mat mask1 = cv::Mat::zeros(face1.rows, face1.cols, CV_8UC1);
			cv::circle(mask1, cv::Point(face1.cols / 2, face1.rows / 2), face1.cols / 4, cv::Scalar::all(255), -1);
			cv::Mat face1_masked;
			face1.copyTo(face1_masked, mask1);
			std::vector<cv::Mat> images = {face0_masked, face1_masked};
			//Eigen::Vector3f pos = dynaman.Sensor()->AffineGlobal2Kinect() * objPtr1->getPosition();
			std::vector<Eigen::Vector3f> positions = { dynaman.Sensor()->AffineGlobal2Kinect() * objPtr1->getPosition() ,dynaman.Sensor()->AffineGlobal2Kinect() * objPtr2->getPosition() };
			//proj.projectImageOnObject(pos, face_masked, cv::Size(300, 300), cv::Scalar::all(0)); // for VR LOGO
			std::vector<cv::Size2f> sizes = { cv::Size(300, 300), cv::Size(300, 300) };
			proj.projectImageOnObject(positions, images, sizes);
			//std::cout << objPtr->getPositionTarget().transpose() << std::endl;
			auto key = cv::waitKey(1);
			if (key == 'q') { break; }
			switch (key) {
			case 'z':
				objPtr1->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, objPtr1->getPosition(), objPtr1->getPositionTarget() - transDist * Eigen::Vector3f::UnitX())));
				break;
			case 'c':
				objPtr1->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, objPtr1->getPosition(), objPtr1->getPositionTarget() + transDist * Eigen::Vector3f::UnitX())));
				break;
			case 's':
				objPtr1->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, objPtr1->getPosition(), objPtr1->getPositionTarget() + transDist * Eigen::Vector3f::UnitZ())));
				break;
			case 'x':
				objPtr1->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, objPtr1->getPosition(), objPtr1->getPositionTarget() - transDist * Eigen::Vector3f::UnitZ())));
				break;
			case 'd':
				objPtr1->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, objPtr1->getPosition(), objPtr1->getPositionTarget() + transDist * Eigen::Vector3f::UnitY())));
				break;
			case 'v':
				objPtr1->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, objPtr1->getPosition(), objPtr1->getPositionTarget() - transDist * Eigen::Vector3f::UnitY())));
				break;
			default:
				break;
			}
		}
	}
	dynaman.Close();
	return 0;
}