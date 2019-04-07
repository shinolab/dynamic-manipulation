#include "init_uist.hpp"
#include "odcs.hpp"
#include "projector.hpp"
#include <opencv2/highgui.hpp>
#include <chrono>
#include <iostream>

int main() {

	Eigen::Vector3f posRight(456.14+100, 596.04 + 160.f, 1231.f);
	Eigen::Vector3f posLeft(-443.86f+150, 596.04f + 160.f, 1231.f);
	Eigen::Vector3f posLeft2(-443.86f-50, 596.04f -370.f, 1431.f);
	double translationTime = 8.0;

	odcs dynaman;
	initialize_uist_setup(dynaman);

	FloatingObjectPtr objPtr1 = FloatingObject::Create(posLeft, -0.0001f);
	//FloatingObjectPtr objPtr2 = FloatingObject::Create(posLeft2, -0.0001f);

	dynaman.RegisterObject(objPtr1);
	//dynaman.RegisterObject(objPtr2);
	dynaman.StartControl();

	//start projection thread.
	std::thread th_proj([&]() {
		std::string projectorName = "projector1";
		projector proj(projectorName);
		std::cout << "now loading..." << std::endl;
		cv::Mat img1 = cv::imread("img/guide_conf2.png");
		//cv::Mat img2 = cv::imread("img/guide_cafe2.png");
		std::cout << "images loaded." << std::endl;
		std::vector<cv::Mat> images = { img1 };
		std::vector<cv::Size2f> sizes = { cv::Size2f(100, 100) };
		proj.CreateScreen();
		while (1)
		{
			if (true)//objPtr1->IsTracked() && objPtr2->IsTracked())
			{
				std::vector<Eigen::Vector3f> positions = { dynaman.Sensor()->AffineGlobal2Kinect() * objPtr1->getPosition() //, dynaman.Sensor()->AffineGlobal2Kinect() * objPtr2->getPosition()
			};
				proj.projectImageOnObject(positions, images, sizes, cv::Scalar::all(0)); // for VR LOGO

			}
			auto key = cv::waitKey(1);
			if (key == 'q') { break; }
		}
	});
	
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(20));
	objPtr1->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, posLeft, posRight)));
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds((int)translationTime + 5));
	dynaman.Close();
	th_proj.join();
	return 0;
}