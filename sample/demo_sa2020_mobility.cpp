#include "CameraDevice.hpp"
#include "StereoTracker.hpp"
#include "additionalGain.hpp"
#include "haptic_icon.hpp"
#include "geometryUtil.hpp"
#include "haptic_icon_strategies/multiplex_strategy.hpp"
#include "haptic_icon_strategies/simple_strategy.hpp"
#include "odcs.hpp"
#include <fstream>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {

	std::cout << "Enter a log file name: ";
	std::string filename;
	std::getline(std::cin, filename);
	//std::string target_image_name("blue_target_cover.png");
	std::string target_image_name("blue_target_no_cover.png");
	std::string leftCamId("32434751");
	std::string rightCamId("43435351");
	Eigen::Vector3f pos_sensor(-125.652f, -871.712f, 13.3176f);
	Eigen::Quaternionf quo_sensor(0.695684f, -0.718283f, -0.0089647f, 0.00359883f);
	Eigen::Vector3f sensor_bias(0.0f, 0.0f, 0.0f);
	int lowerb = 10, upperb = 255, hist_size = 30;
	std::cout << "loading target images ..." << std::endl;
	cv::Mat img_target = cv::imread(target_image_name);
	if (img_target.empty()) {
		std::cerr << "Failed to open the target image." << std::endl;
	}
	std::cout << "initializing an extractor ..." << std::endl;
	std::vector<cv::Mat> imgs_target = { img_target };
	auto extractorPtrLeft = imgProc::hue_backproject_extractor::create(imgs_target, lowerb, upperb, hist_size);
	auto extractorPtrRight = imgProc::hue_backproject_extractor::create(imgs_target, lowerb, upperb, hist_size);

	std::cout << "initializing a camera ..." << std::endl;
	auto leftCamPtr = ximeaCameraDevice::create(leftCamId);
	auto rightCamPtr = ximeaCameraDevice::create(rightCamId);
	auto stereoCamPtr = dynaman::stereoCamera::create(leftCamPtr, rightCamPtr);
	std::cout << "opening cameras..." << std::endl;
	stereoCamPtr->open();
	std::cout << "initializing a tracker..." << std::endl;
	dynaman::stereoTracker tracker(stereoCamPtr, extractorPtrLeft, extractorPtrRight, pos_sensor, quo_sensor, sensor_bias);

	dynaman::odcs manipulator(tracker);
	haptic_icon::Initialize(manipulator);
	Eigen::Vector3f gainP = 20 * Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = 5 * Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = 1 * Eigen::Vector3f::Constant(-0.05f);
	Eigen::Vector3f pos_init(0, -50.f, 0);
	manipulator.ocsPtr->SetGain(gainP, gainD, gainI);
	auto objPtr = dynaman::FloatingObject::Create(
		pos_init,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		-0.036e-3f,
		50.f);
	std::cout << "workspace " << std::endl
		<< "lower bound: " << objPtr->lowerbound().transpose() << std::endl
		<< "upper bound: " << objPtr->upperbound().transpose() << std::endl;

	int duration = 60000;//1800000; 
	int loopPeriod = 10;
	int freq = 100;
	float lambda = 0;
	
	//dynaman::simple_strategy strategy(gainP, gainD, gainI, loopPeriod, filename, 1.0f);
	dynaman::multiplex_strategy strategy(gainP, gainD, gainI, loopPeriod, filename, freq, lambda);

	std::thread th_control([&manipulator, &strategy, &objPtr, &duration]() {
		strategy.run(manipulator, objPtr, duration);
		}
	);
	
	Eigen::Vector3f posCenter(0.f, -50.f, 0.f);
	Eigen::Vector3f posRight(300.f, -50.f, 0.f);
	Eigen::Vector3f posLeft(-300.f, -50.f, 0.f);
	Eigen::Vector3f posHigh(0.f, -50.f, 300.f);
	Eigen::Vector3f posLow(0.f, -50.f, -250.f);
	// params for circular trajectory
	float orbit_radius = 150;
	float orbit_period = 3.f;
	Eigen::Vector3f posCircleInit(orbit_radius, 0.f, 0.f);
	float timeTrans = 2.0f;
	//manipulator.ocsPtr->SetGain(gainP, 1 * 5 * Eigen::Vector3f::Constant(-4.0f), gainI);

	std::this_thread::sleep_for(std::chrono::seconds(5));

	//std::getline(std::cin, std::string());
	//objPtr->updateStatesTarget(objPtr->getPosition());
	//std::getline(std::cin, std::string());
	//objPtr->updateStatesTarget(objPtr->getPosition());

	//traslation maneuver #1:
	objPtr->updateStatesTarget(posLeft);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posLeft));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	objPtr->updateStatesTarget(posRight);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(3.0f, timeGetTime(), posLeft, posRight));
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	objPtr->updateStatesTarget(posCenter);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posRight, posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	objPtr->updateStatesTarget(posHigh);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posHigh));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	objPtr->updateStatesTarget(posLow);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(3.0f, timeGetTime(), posHigh, posLow));
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
	objPtr->updateStatesTarget(posCenter);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posLow, posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	//circular maneuver:
	manipulator.ocsPtr->SetGain(gainP, 1 * 5 * Eigen::Vector3f::Constant(-4.0f), gainI);
	objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posCircleInit));
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	objPtr->SetTrajectory(dynaman::TrajectoryCircle::Create(posCenter, orbit_radius, pi / 2.f, 0.f, orbit_period, 0.f, timeGetTime()));
	std::this_thread::sleep_for(std::chrono::milliseconds(6000));
	objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime(), objPtr->getPosition(), posCenter));
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));

	////traslation maneuver #2:
	//float triangle_length = 300;
	//float x_tri = 0.5 * triangle_length;
	//float bottom_tri = triangle_length * cosf(M_PI / 3) / 3;
	//float top_tri = triangle_length * cosf(M_PI / 3) * 2 / 3;
	//Eigen::Vector3f pt = Eigen::Vector3f(0, 0, top_tri) + posCenter;
	//Eigen::Vector3f ptr = Eigen::Vector3f(x_tri, 0, bottom_tri) + posCenter;
	//Eigen::Vector3f ptl = Eigen::Vector3f(-x_tri, 0, bottom_tri) + posCenter;
	//Eigen::Vector3f pb = Eigen::Vector3f(0, 0, -top_tri) + posCenter;
	//Eigen::Vector3f pbr = Eigen::Vector3f(x_tri, 0, -bottom_tri) + posCenter;
	//Eigen::Vector3f pbl = Eigen::Vector3f(-x_tri, 0, -bottom_tri) + posCenter;
	//Eigen::Vector3f posCenter2(0.f, -50.f, 0.f);
	//Eigen::Vector3f posRight2(200.f, -50.f, 0.f);
	//Eigen::Vector3f posLeft2(-200.f, -50.f, 0.f);
	//Eigen::Vector3f posHigh2(0.f, -50.f, 200.f);
	//Eigen::Vector3f posLow2(0.f, -50.f, -250.f);

	////first triangle
	//manipulator.ocsPtr->SetGain(gainP, 10 * Eigen::Vector3f::Constant(-4.0f), gainI);

	//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter2, 0.8*posLeft2));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), 0.8*posLeft2, posCenter2));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter2, Eigen::Vector3f(0, 100, 0)));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), Eigen::Vector3f(0, 100, 0), posCenter2));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter2, posHigh2));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posHigh2, posCenter2));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//objPtr->updateStatesTarget(posCenter);//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posLow, posCenter));
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	//circular maneuver:
	//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posCircleInit));
	//std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	//objPtr->SetTrajectory(dynaman::TrajectoryCircle::Create(posCenter, orbit_radius, pi / 2.f, 0.f, orbit_period, 0.f, timeGetTime()));
	//std::this_thread::sleep_for(std::chrono::milliseconds(6000));
	//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime(), objPtr->getPosition(), posCenter));
	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	//std::cout << "Applying InfShapeTrajectory" << std::endl;
	//manipulator.ocsPtr->SetGain(gainP, 5 * Eigen::Vector3f::Constant(-4.0f), gainI);
	////objPtr->updateStatesTarget(Eigen::Vector3f(0, 0, 150));//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(2.0f, timeGetTime(), posCenter, posLeft));
	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	////objPtr->SetTrajectory(dynaman::TrajectoryCircle::Create(posCenter+ Eigen::Vector3f(-100, 0, 0), orbit_radius/2, pi / 2.f, 0.f, 1.5*orbit_period, 0.f, timeGetTime()));
	//objPtr->SetTrajectory(dynaman::TrajectoryInfShape::Create(posCenter, 200, 400, 6*orbit_period, timeGetTime()));
	////objPtr->SetTrajectory(dynaman::TrajectoryHeart::Create(posCenter, 100, 300, 5*orbit_period, timeGetTime()));
	//std::this_thread::sleep_for(std::chrono::milliseconds(60000));
	//std::cout << "Eight finished." << std::endl;
	//objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(1.0f, timeGetTime(), objPtr->getPosition(), posCenter));
	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));

	th_control.join();
	manipulator.Close();

	return 0;
}