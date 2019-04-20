#include "init_uist.hpp"
#include "odcs.hpp"
#include "projector.hpp"
#include <opencv2/highgui.hpp>
#include <chrono>
#include <vector>
#include <thread>
#include <iostream>

int main() {

	Eigen::Vector3f posRight(356.14-150, 596.04+100, 1331.f-100);
	Eigen::Vector3f posLeft(-343.86f-50, 596.04f+100, 1331.f-100);

	double translationTime = 10.0;

	odcs dynaman;
	initialize_uist_setup(dynaman);

	FloatingObjectPtr objPtr1 = FloatingObject::Create(posRight, -0.0001f);
	//FloatingObjectPtr objPtr2 = FloatingObject::Create(posLeft, -0.0001f);
	dynaman.RegisterObject(objPtr1);
	//dynaman.RegisterObject(objPtr2);
	dynaman.StartControl();
	std::cout << "control started." << std::endl;
	//start projection thread.
	std::thread th_proj([&]() {
		std::string projectorName = "projector1";
		projector proj(projectorName);
		std::cout << "now loading..." << std::endl;
		cv::Mat img1 = cv::imread("img/instruction1.png");
		cv::Mat img2 = cv::imread("img/instruction2.png");
		cv::Mat img = cv::imread("img/earth_sphere.png");
		std::cout << "images loaded." << std::endl;
		std::vector<cv::Mat> images = { img };
		std::vector<cv::Size2f> sizes = {cv::Size2f(160, 160)};
		proj.CreateScreen();
		while (1)
		{
			if (objPtr1->IsTracked())// && objPtr2->IsTracked())
			{
				std::vector<Eigen::Vector3f> positions = { dynaman.Sensor()->AffineGlobal2Kinect() * objPtr1->getPosition() };//, dynaman.Sensor()->AffineGlobal2Kinect() * objPtr2->getPosition()
			
				proj.projectImageOnObject(positions, images, sizes, cv::Scalar::all(0)); // for VR LOGO

			}
			auto key = cv::waitKey(1);
			if (key == 'q') { break; }
		}
	});

	th_proj.join();
	dynaman.Close();
	return 0;
}
