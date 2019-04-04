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
	cv::VideoCapture cap(0);
	if (!cap.isOpened()) {
		std::cerr << "ERROR: failed to open camera." << std::endl;
		return -1;
	}

	odcs dynaman;
	initialize_uist_setup(dynaman);

	FloatingObjectPtr objPtr = FloatingObject::Create(Eigen::Vector3f(436.14, 596.04, 1331.f), -0.0001f);
	dynaman.RegisterObject(objPtr);
	dynaman.StartControl();

	while (1) {
		if (true)
		{
			cv::Mat face;
			cap.read(face);
			cv::Mat mask = cv::Mat::zeros(face.rows, face.cols, CV_8UC1);
			cv::circle(mask, cv::Point(face.cols / 2, face.rows / 2), face.cols / 4, cv::Scalar::all(255), -1);
			cv::Mat face_masked;
			face.copyTo(face_masked, mask);
			Eigen::Vector3f pos = dynaman.Sensor()->AffineGlobal2Kinect() * objPtr->getPosition();
			proj.projectImageOnObject(pos, face_masked, cv::Size(300, 300), cv::Scalar::all(0)); // for VR LOGO
			std::cout << objPtr->getPositionTarget().transpose() << std::endl;
			auto key = cv::waitKey(1);
			if (key == 'q') { break; }
			switch (key) {
			case 'z':
				objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, objPtr->getPosition(), objPtr->getPositionTarget() - transDist * Eigen::Vector3f::UnitX())));
				break;
			case 'c':
				objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, objPtr->getPosition(), objPtr->getPositionTarget() + transDist * Eigen::Vector3f::UnitX())));
				break;
			case 's':
				objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, objPtr->getPosition(), objPtr->getPositionTarget() + transDist * Eigen::Vector3f::UnitZ())));
				break;
			case 'x':
				objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, objPtr->getPosition(), objPtr->getPositionTarget() - transDist * Eigen::Vector3f::UnitZ())));
				break;
			case 'd':
				objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, objPtr->getPosition(), objPtr->getPositionTarget() + transDist * Eigen::Vector3f::UnitY())));
				break;
			case 'v':
				objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, objPtr->getPosition(), objPtr->getPositionTarget() - transDist * Eigen::Vector3f::UnitY())));
				break;
			default:
				break;
			}
		}
	}
	dynaman.Close();
	return 0;
}