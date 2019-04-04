#include "init_uist.hpp"
#include "odcs.hpp"
#include "projector.hpp"
#include <opencv2/highgui.hpp>
#include <chrono>
#include <iostream>

int main() {

	Eigen::Vector3f posRight(456.14, 596.04, 1331.f);
	Eigen::Vector3f posLeft(-443.86f, 596.04f, 1331.f);
	double translationTime = 10.0;

	odcs dynaman;
	initialize_uist_setup(dynaman);

	FloatingObjectPtr objPtr = FloatingObject::Create(Eigen::Vector3f(456.14, 596.04, 1331.f), -0.0001f);
	dynaman.RegisterObject(objPtr);
	dynaman.StartControl();

	//start projection thread.
	std::thread th_proj([&]() {
		std::string projectorName = "projector1";
		projector proj(projectorName);
		cv::Mat img = cv::imread("img/follow_me.png");
		proj.CreateScreen();
		while (1)
		{
			if (objPtr->IsTracked())
			{
				Eigen::Vector3f pos = dynaman.Sensor()->AffineGlobal2Kinect() * objPtr->getPosition();
				std::cout << pos.x() << ", " << pos.y() << ", " << pos.z() << std::endl;
				proj.projectImageOnObject(pos, img, cv::Size(160, 160), cv::Scalar::all(0)); // for VR LOGO

				auto key = cv::waitKey(1);
				if (key == 'q') { break; }
			}
		}
	});
	
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(10));
	objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBangBang(translationTime, timeGetTime() / 1000.f, posRight, posLeft)));
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds((int)translationTime + 5));
	dynaman.Close();
	th_proj.join();
	return 0;
}