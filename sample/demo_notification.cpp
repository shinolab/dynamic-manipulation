#include "init_uist.hpp"
#include "odcs.hpp"
#include "projector.hpp"
#include <opencv2/highgui.hpp>
#include <chrono>
#include <thread>
#include <iostream>

int main() {

	cv::Mat img1 = cv::imread("img/weekly_meeting.png");
	cv::Mat img2 = cv::imread("img/follow_me.png");

	odcs dynaman;
	initialize_uist_setup(dynaman);

	FloatingObjectPtr objPtr = FloatingObject::Create(Eigen::Vector3f(-423.86f, 596.04f, 1331.f), -0.0001f);
	dynaman.RegisterObject(objPtr);
	dynaman.StartControl();

	std::string projectorName = "projector1";
	projector proj(projectorName);
	proj.setImage(cv::Mat::zeros(300, 300, CV_8UC3));
	//start projection thread.
	std::thread th_proj([&]() {	
		proj.CreateScreen();
		while (1)
		{
			if (objPtr->IsTracked())
			{
				Eigen::Vector3f pos = dynaman.Sensor()->AffineGlobal2Kinect() * objPtr->getPosition();
				proj.projectImageOnObject(pos, cv::Size(180, 180), cv::Scalar::all(0)); // for VR LOGO

				auto key = cv::waitKey(1);
				if (key == 'q') { break; }
			}
		}
	});

	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(20));
	proj.setImage(img1);
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(3));

	bool tracking_succeeded = false;
	do
	{
		HRESULT hr = dynaman.Sensor()->kinect()->getBodies();
		if (SUCCEEDED(hr)) {
			for (auto itr = dynaman.Sensor()->kinect()->pBodies.begin(); itr != dynaman.Sensor()->kinect()->pBodies.end(); itr++)
			{
				BOOLEAN isTracked = false;
				HRESULT hrTrack = (*itr)->get_IsTracked(&isTracked);
				if (isTracked && SUCCEEDED(hrTrack))
				{
					std::vector<Joint> joints(JointType::JointType_Count);
					HRESULT hrJoints = (*itr)->GetJoints(JointType::JointType_Count, &joints[0]);
					if (SUCCEEDED(hrJoints)) {
						CameraSpacePoint cspHead = joints[JointType::JointType_Head].Position;
						Eigen::Vector3f posHead = dynaman.Sensor()->AffineKinect2Global() * Eigen::Vector3f(1000 * cspHead.X, 1000 * cspHead.Y, 1000 * cspHead.Z);
						if (dynaman.Sensor()->isInsideWorkSpace(posHead)) {
							tracking_succeeded = true;
							std::cout << "User Found" << std::endl;
							objPtr->SetTrajectory(std::shared_ptr<Trajectory>(new TrajectoryBang(3.f, timeGetTime() / 1000.f, objPtr->getPositionTarget(), posHead)));
						}
					}
				}
			}
		}
		Sleep(10);
	} while (!tracking_succeeded);

	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(13));
	proj.setImage(cv::Mat::zeros(300, 300, CV_8UC3));
	th_proj.join();
	dynaman.Close();
	return 0;
}