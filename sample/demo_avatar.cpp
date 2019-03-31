#include "init_uist.hpp"
#include "odcs.hpp"
#include "projector.hpp"
#include "opencv2/highgui.hpp"
#include <chrono>
#include <thread>
#include <iostream>

#include <Windows.h>

#pragma comment(lib, "winmm")

int main() {

	float translationTime = 1.0f;
	float transDist = 50.f;

	std::string projectorName = "projector1";
	projector proj(projectorName);
	proj.CreateScreen();
	cv::VideoCapture cap(0);
	if (!cap.isOpened()) {
		std::cerr << "ERROR: failed to open camera." << std::endl;
		return -1;
	}

	while (1) {
		cv::Mat face;
		cap.read(face);
		cv::imshow("cam", face);
		if (cv::waitKey(1) == 'q') {
			break;
		}
	}

	return 0;

	odcs dynaman;
	initialize_uist_setup(dynaman);

	FloatingObjectPtr objPtr = FloatingObject::Create(Eigen::Vector3f(456.14, 596.04, 1331.f), -0.0001f);
	dynaman.RegisterObject(objPtr);
	dynaman.StartControl();

	while (1) {
		if (objPtr->IsTracked())
		{
			cv::Mat face;
			cap.read(face);
			Eigen::Vector3f pos = dynaman.Sensor()->AffineGlobal2Kinect() * objPtr->AveragePosition();
			proj.projectImageOnObject(pos, face, cv::Size(180, 180), cv::Scalar::all(0)); // for VR LOGO

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