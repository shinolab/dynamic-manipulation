#include "CameraDevice.hpp"
#include "StereoTracker.hpp"
#include "additionalGain.hpp"
#include "KinectSphereTracker.hpp"
#include "haptic_icon.hpp"
#include "geometryUtil.hpp"
#include "odcs.hpp"
#include <fstream>
#include <chrono>
#include <thread>

namespace dynaman {
	class balance_control_strategy {
		Eigen::Vector3f _gainP;
		Eigen::Vector3f _gainD;
		Eigen::Vector3f _gainI;
		int _loopPeriod;
		std::string _fileName;
		float _focusBlur;
	public:
		balance_control_strategy(
			const Eigen::Vector3f &gainP,
			const Eigen::Vector3f &gainD,
			const Eigen::Vector3f &gainI,
			const int loopPeriod,
			const std::string &fileName,
			const float focusBlur=1.f)
			:_gainP(gainP),
			_gainD(gainD),
			_gainI(gainI),
			_loopPeriod(loopPeriod),
			_fileName(fileName)
			,_focusBlur(focusBlur){};
		~balance_control_strategy() {}
		
		int run(dynaman::odcs& manipulator, FloatingObjectPtr objPtr, int duration) {
			std::ofstream ofs_config(_fileName + "_config.txt");
			if (!ofs_config.is_open()) {
				std::cerr << "Failed to open the config file." << std::endl;
				return 1;
			}

			ofs_config
				<< "balance_control_strategy config" << std::endl
				<< "gainP: " << _gainP.transpose() << std::endl
				<< "gainD: " << _gainD.transpose() << std::endl
				<< "gainI: " << _gainI.transpose() << std::endl
				<< "loopPeriod: " << _loopPeriod << std::endl
				<< "centersAUTD: " << std::endl << manipulator.Controller()->CentersAUTD().transpose() << std::endl
				<< "positionsAUTD: " << std::endl << manipulator.Controller()->positionsAUTD.transpose() << std::endl
				<< "directionsAUTD: " << std::endl << manipulator.Controller()->DirectionsAUTD().transpose() << std::endl
				<< "focusBlur: " << _focusBlur;

			std::ofstream ofs(_fileName + ".csv");
			if (!ofs.is_open()) {
				std::cerr << "Failed to open log file." << std::endl;
				return 1;
			}

			ofs << "t, x, y, z, vx, vy, vz, ix, iy, iz, xTgt, yTgt, zTgt,"
				<< "axTgt, ayTgt, azTgt, fxTgt, fyTgt, fzTgt, fxRes, fyRes, fzRes, "
				<< "u0(pxpy), u1(px), u2(pxmy), u3(mxmy), u4(mx), u5(mxpy)"
				<< "u6(mxpy), u7(mxmy), u8(pxmy), u9(pxpy), u10(mz)"
				<< std::endl;
			DWORD initTime = timeGetTime();
			while (timeGetTime() - initTime < duration) {
				DWORD loopInit = timeGetTime();
				//----------Observation----------
				Eigen::Vector3f posObserved;
				DWORD observationTime;
				bool observed = manipulator.sensor.observe(observationTime, posObserved, objPtr);
				//std::cout << "running" << std::endl;
				//std::cout << observed << ", " << isInsideWorkspace(posObserved, (*itr)->lowerbound(), (*itr)->upperbound()) << std::endl;
				if (observed && isInsideWorkspace(posObserved, objPtr->lowerbound(), objPtr->upperbound())) {
					//std::cout << "observed." << std::endl;
					objPtr->updateStates(observationTime, posObserved);
					objPtr->SetTrackingStatus(true);
					//Create Gain:
					auto vel = objPtr->averageVelocity();
					auto integ = objPtr->getIntegral();
					auto posTgt = objPtr->getPositionTarget();
					Eigen::Vector3f accelTgt
						= _gainP.asDiagonal() * (objPtr->getPosition() - objPtr->getPositionTarget())
						+ _gainD.asDiagonal() * (objPtr->averageVelocity() - objPtr->getVelocityTarget())
						+ _gainI.asDiagonal() * objPtr->getIntegral()
						+ objPtr->getAccelTarget();
					Eigen::Vector3f forceToApply = objPtr->totalMass() * accelTgt + objPtr->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665e3f);
					Eigen::VectorXf duties = manipulator.Controller()->FindDutyQPCGAL(forceToApply, objPtr->getPosition());
					Eigen::VectorXi amplitudes = (510.f / M_PI * duties.array().max(0.f).min(1.f).sqrt().asin().matrix()).cast<int>();
					Eigen::MatrixXf centersAutd = manipulator.Controller()->CentersAUTD();
					Eigen::Vector3f forceResult = manipulator.Controller()->arfModelPtr->arf(
						posObserved.replicate(1, manipulator.Controller()->_autd.geometry()->numDevices()) - centersAutd,
						manipulator.Controller()->eulerAnglesAUTD)
						* duties;
					Eigen::MatrixXf focus = centersAutd + _focusBlur * (objPtr->getPosition().replicate(1, centersAutd.cols()) - centersAutd);
					auto gain = autd::DeviceSpecificFocalPointGain::Create(focus, amplitudes);
					manipulator.Controller()->_autd.AppendGainSync(gain);
					ofs << observationTime << ", " << posObserved.x() << ", " << posObserved.y() << ", " << posObserved.z() << ", "
						<< vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
						<< integ.z() << ", " << integ.y() << ", " << integ.z() << ", "
						<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", "
						<< accelTgt.x() << ", " << accelTgt.y() << ", " << accelTgt.z() << ","
						<< forceToApply.x() << ", " << forceToApply.y() << ", " << forceToApply.z() << ", "
						<< forceResult.x() << ", " << forceResult.y() << ", " << forceResult.z() << ", "
						<< duties(0) << ", " << duties(1) << ", " << duties(2) << ", " << duties(3) << ", " << duties(4) << ", " << duties(5) << ", "
						<< duties(6) << ", " << duties(7) << ", " << duties(8) << ", " << duties(9) << ", " << duties(10)
						<< std::endl;
				}
				else if (loopInit - objPtr->lastDeterminationTime > 1000)
				{
					objPtr->SetTrackingStatus(false);
				}
				int waitTime = _loopPeriod - (timeGetTime() - loopInit);
				timeBeginPeriod(1);
				Sleep(std::max(waitTime, 0));
				timeEndPeriod(1);
			}
			ofs.close();
			return 0;
		};
		
	};
	
	class simple_strategy {
		Eigen::Vector3f _gainP;
		Eigen::Vector3f _gainD;
		Eigen::Vector3f _gainI;
		int _loopPeriod;
		std::string _fileName;
		float _focusBlur;
	public:
		simple_strategy(const Eigen::Vector3f& gainP,
			const Eigen::Vector3f& gainD,
			const Eigen::Vector3f& gainI,
			const int loopPeriod,
			const std::string& fileName,
			const float focusBlur)
			:_gainP(gainP),
			_gainD(gainD),
			_gainI(gainI),
			_loopPeriod(loopPeriod),
			_fileName(fileName),
			_focusBlur(focusBlur){}
		
		~simple_strategy() {}

		int run(dynaman::odcs& manipulator, FloatingObjectPtr objPtr, int duration) {
			std::ofstream ofs_config(_fileName + "_config.txt");
			if (!ofs_config.is_open()) {
				std::cerr << "Failed to open the config file." << std::endl;
				return 1;
			}
			ofs_config
				<< "simple_strategy config" << std::endl
				<< "gainP: " << _gainP.transpose() << std::endl
				<< "gainD: " << _gainD.transpose() << std::endl
				<< "gainI: " << _gainI.transpose() << std::endl
				<< "loopPeriod: " << _loopPeriod << std::endl
				<< "centersAUTD:" << std::endl << manipulator.Controller()->CentersAUTD().transpose() << std::endl
				<< "positionsAUTD:" << std::endl << manipulator.Controller()->positionsAUTD.transpose() << std::endl
				<< "directionsAUTD:" << std::endl << manipulator.Controller()->DirectionsAUTD().transpose() << std::endl
				<< "focusBlur: " << _focusBlur;

			std::ofstream ofs(_fileName + ".csv");
			if (!ofs.is_open()) {
				std::cerr << "Failed to open log file." << std::endl;
				return 1;
			}

			ofs << "t, x, y, z, vx, vy, vz, ix, iy, iz, xTgt, yTgt, zTgt,"
				<< "axTgt, ayTgt, azTgt, fxTgt, fyTgt, fzTgt, fxRes, fyRes, fzRes, "
				<< "u0(pxpy), u1(px), u2(pxmy), u3(mxmy), u4(mx), u5(mxpy),"
				<< "u6(mxpy), u7(mxmy), u8(pxmy), u9(pxpy), u10(mz)"
				<< std::endl;
			DWORD initTime = timeGetTime();
			while (timeGetTime() - initTime < duration) {
				DWORD loopInit = timeGetTime();
				//----------Observation----------
				Eigen::Vector3f posObserved;
				DWORD observationTime;
				bool observed = manipulator.sensor.observe(observationTime, posObserved, objPtr);
				//std::cout << "running" << std::endl;
				//std::cout << observed << ", " << isInsideWorkspace(posObserved, (*itr)->lowerbound(), (*itr)->upperbound()) << std::endl;
				if (observed && isInsideWorkspace(posObserved, objPtr->lowerbound(), objPtr->upperbound())) {
					//std::cout << "observed." << std::endl;
					objPtr->updateStates(observationTime, posObserved);
					objPtr->SetTrackingStatus(true);
					//Create Gain:
					auto vel = objPtr->averageVelocity();
					auto integ = objPtr->getIntegral();
					auto posTgt = objPtr->getPositionTarget();
					Eigen::Vector3f accelTgt
						= _gainP.asDiagonal() * (objPtr->getPosition() - objPtr->getPositionTarget())
						+ _gainD.asDiagonal() * (objPtr->averageVelocity() - objPtr->getVelocityTarget())
						+ _gainI.asDiagonal() * objPtr->getIntegral()
						+ objPtr->getAccelTarget();
					Eigen::Vector3f forceTgt = objPtr->totalMass() * accelTgt + objPtr->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665e3f);
					Eigen::MatrixXf centersAutd = manipulator.Controller()->CentersAUTD();
					Eigen::MatrixXf posRel = posObserved.replicate(1, manipulator.Controller()->_autd.geometry()->numDevices()) - centersAutd;
					Eigen::MatrixXf directionsArf = posRel.colwise().normalized();
					Eigen::MatrixXf innerProducts = forceTgt.transpose() * directionsArf;
					Eigen::MatrixXf::Index row, active_id;
					auto forceToApplyAbs = innerProducts.maxCoeff(&row, &active_id);
					Eigen::Vector3f forceToApply = forceToApplyAbs * directionsArf.col(active_id);
					float forceMax = manipulator.Controller()->arfModelPtr->arf(
						posRel.col(active_id),
						manipulator.Controller()->eulerAnglesAUTD.col(active_id)
						).norm();
					Eigen::VectorXf duties = Eigen::VectorXf::Zero(manipulator.Controller()->_autd.geometry()->numDevices());
					duties(active_id) = forceToApplyAbs / forceMax;
					Eigen::VectorXi amplitudes = (510.f / M_PI * duties.array().max(0.f).min(1.f).sqrt().asin().matrix()).cast<int>();
					Eigen::Vector3f forceResult = manipulator.Controller()->arfModelPtr->arf(
						posRel,
						manipulator.Controller()->eulerAnglesAUTD
						) * duties;
					Eigen::MatrixXf focus = centersAutd + _focusBlur * (objPtr->getPosition().replicate(1, centersAutd.cols()) - centersAutd);
					auto gain = autd::DeviceSpecificFocalPointGain::Create(focus, amplitudes);
					manipulator.Controller()->_autd.AppendGainSync(gain);
					manipulator.Controller()->_autd.AppendModulationSync(autd::Modulation::Create(255));
					ofs << observationTime << ", " << posObserved.x() << ", " << posObserved.y() << ", " << posObserved.z() << ", "
						<< vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
						<< integ.z() << ", " << integ.y() << ", " << integ.z() << ", "
						<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", "
						<< accelTgt.x() << ", " << accelTgt.y() << ", " << accelTgt.z() << ","
						<< forceToApply.x() << ", " << forceToApply.y() << ", " << forceToApply.z() << ", "
						<< forceResult.x() << ", " << forceResult.y() << ", " << forceResult.z() << ", "
						<< duties(0) << ", " << duties(1) << ", " << duties(2) << ", " << duties(3) << ", " << duties(4) << ", " << duties(5) << ", "
						<< duties(6) << ", " << duties(7) << ", " << duties(8) << ", " << duties(9) << ", " << duties(10)
						<< std::endl;
				}
				else if (loopInit - objPtr->lastDeterminationTime > 1000)
				{
					objPtr->SetTrackingStatus(false);
				}
				int waitTime = _loopPeriod - (timeGetTime() - loopInit);
				timeBeginPeriod(1);
				Sleep(std::max(waitTime, 0));
				timeEndPeriod(1);
			}
			ofs.close();
			return 0;
		}
	};
}

int main(int argc, char** argv) {

	std::string filename("20200408_balance_strategy_blurx1_2");
	std::string target_image_name("blue_target_cover.png");
	std::string leftCamId("32434751");
	std::string rightCamId("43435351");
	Eigen::Vector3f pos_sensor(-125.652, -871.712, 13.3176);
	Eigen::Quaternionf quo_sensor(0.695684, -0.718283, -0.0089647, 0.00359883);
	Eigen::Vector3f sensor_bias(0.0f, 0.0f, 0.0f);
	int lowerb = 10, upperb = 255, hist_size = 30;
	std::cout << "loading target images ..." << std::endl;
	cv::Mat img_target = cv::imread(target_image_name);
	if (img_target.empty()) {
		std::cerr << "Failed to open the target image." << std::endl;
	}
	std::cout << "initializing an extractor ..." << std::endl;
	//initialize tracking algorithm
	std::vector<cv::Mat> imgs_target = { img_target };
	auto extractorPtrLeft = imgProc::hue_backproject_extractor::create(imgs_target, lowerb, upperb, hist_size);
	auto extractorPtrRight = imgProc::hue_backproject_extractor::create(imgs_target, lowerb, upperb, hist_size);

	//initialize camera
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
	//haptic_icon::InitializeLower(manipulator);
	Eigen::Vector3f gainP = 10* Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = 10* Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = 1* Eigen::Vector3f::Constant(-0.05f);
	manipulator.ocsPtr->SetGain(gainP, gainD, gainI);
	auto objPtr = dynaman::FloatingObject::Create(
		Eigen::Vector3f(0, 0, 0),
		Eigen::Vector3f::Constant(-500),
		Eigen::Vector3f::Constant(500),
		0,//4.0e-5f,
		50.f);
	std::cout << "workspace " << std::endl
		<< "lower bound: " << objPtr->lowerbound().transpose() << std::endl
		<< "upper bound: " << objPtr->upperbound().transpose() << std::endl;
	//Eigen::MatrixXf centersAutd = manipulator.Controller()->CentersAUTD();
	int duration = 60000;
	int loopPeriod = 10;
	float focusBlur = 1;
	dynaman::balance_control_strategy strategy(gainP, gainD, gainI, loopPeriod, filename, focusBlur);
	//dynaman::simple_strategy strategy(gainP, gainD, gainI, loopPeriod, filename, focusBlur);
	strategy.run(manipulator, objPtr, duration);
	manipulator.Close();

	return 0;
}