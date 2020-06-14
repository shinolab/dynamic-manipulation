#include "MonitorInfoManager.hpp"
#include "DrawUtil.hpp"
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
#include <random>

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
				<< "u0(pxpy), u1(px), u2(pxmy), u3(mxmy), u4(mx), u5(mxpy),"
				<< "u6(mxpy), u7(mxmy), u8(pxmy), u9(pxpy), u10(mz)"
				<< std::endl;
			DWORD initTime = timeGetTime();
			while (timeGetTime() - initTime < duration) {
				DWORD loopInit = timeGetTime();
				Eigen::Vector3f posObserved;
				DWORD observationTime;
				bool observed = manipulator.sensor.observe(observationTime, posObserved, objPtr);
				if (observed && isInsideWorkspace(posObserved, objPtr->lowerbound(), objPtr->upperbound())) {
					objPtr->updateStates(observationTime, posObserved);
					objPtr->SetTrackingStatus(true);
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
						<< integ.x() << ", " << integ.y() << ", " << integ.z() << ", "
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
		simple_strategy(
			const Eigen::Vector3f& gainP,
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
						<< integ.x() << ", " << integ.y() << ", " << integ.z() << ", "
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

	class multiplex_strategy {
		Eigen::Vector3f _gainP;
		Eigen::Vector3f _gainD;
		Eigen::Vector3f _gainI;
		std::string _fileName;
		int _loopPeriod;
		float _freq;
		float _lambda;
	public:
		multiplex_strategy(
			const Eigen::Vector3f& gainP,
			const Eigen::Vector3f& gainD,
			const Eigen::Vector3f& gainI,
			int loopPeriod,
			const std::string& fileName,
			int freq,
			float lambda)
			:_gainP(gainP),
			_gainD(gainD),
			_gainI(gainI),
			_loopPeriod(loopPeriod),
			_fileName(fileName),
			_freq(freq),
			_lambda(lambda){}
		
		~multiplex_strategy() {}

		int run(dynaman::odcs &manipulator, FloatingObjectPtr objPtr, int duration) {
			std::ofstream ofs_config(_fileName + "_config.txt");
			if (!ofs_config.is_open()) {
				std::cerr << "Failed to open the config file." << std::endl;
				return 1;
			}
			ofs_config
				<< "multiplex_strategy config" << std::endl
				<< "gainP: " << _gainP.transpose() << std::endl
				<< "gainD: " << _gainD.transpose() << std::endl
				<< "gainI: " << _gainI.transpose() << std::endl
				<< "loopPeriod: " << _loopPeriod << std::endl
				<< "centersAUTD:" << std::endl << manipulator.Controller()->CentersAUTD().transpose() << std::endl
				<< "positionsAUTD:" << std::endl << manipulator.Controller()->positionsAUTD.transpose() << std::endl
				<< "directionsAUTD:" << std::endl << manipulator.Controller()->DirectionsAUTD().transpose() << std::endl
				<< "freq: " << _freq;

			std::ofstream ofs(_fileName + ".csv");
			if (!ofs.is_open()) {
				std::cerr << "Failed to open log file." << std::endl;
				return 1;
			}

			ofs << "t, x, y, z, vx, vy, vz, ix, iy, iz, xTgt, yTgt, zTgt, "
				<< "vxTgt, vyTgt, vzTgt, "
				<< "axTgt, ayTgt, azTgt, fxTgt, fyTgt, fzTgt, fxRes, fyRes, fzRes, "
				<< "u0(pxpy), u1(px), u2(pxmy), u3(mxmy), u4(mx), u5(mxpy),"
				<< "u6(mxpy), u7(mxmy), u8(pxmy), u9(pxpy), u10(mz)"
				<< std::endl;
			DWORD initTime = timeGetTime();
			while (timeGetTime() - initTime < duration) {
				DWORD loopInit = timeGetTime();
				Eigen::Vector3f posObserved;
				DWORD observationTime;
				bool observed = manipulator.sensor.observe(observationTime, posObserved, objPtr);
				if (observed && isInsideWorkspace(posObserved, objPtr->lowerbound(), objPtr->upperbound())) {
					objPtr->updateStates(observationTime, posObserved);
					objPtr->SetTrackingStatus(true);
					auto vel = objPtr->averageVelocity();
					auto integ = objPtr->getIntegral();
					auto posTgt = objPtr->getPositionTarget();
					auto velTgt = objPtr->getVelocityTarget();
					const int num_autd = manipulator.Controller()->_autd.geometry()->numDevices();
					Eigen::Vector3f accelTgt
						= _gainP.asDiagonal() * (objPtr->getPosition() - posTgt)
						+ _gainD.asDiagonal() * (objPtr->averageVelocity() - velTgt)
						+ _gainI.asDiagonal() * objPtr->getIntegral()
						+ objPtr->getAccelTarget();
					Eigen::Vector3f forceToApply = objPtr->totalMass() * accelTgt + objPtr->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665e3f);
					Eigen::VectorXf duties = manipulator.Controller()->FindDutyQpMultiplex(forceToApply, objPtr->getPosition(), _lambda);
					std::vector<float> dutiesStl(duties.size());
					Eigen::Map<Eigen::VectorXf>(&dutiesStl[0], duties.size()) = duties;
					int num_active = (duties.array() > 1.0e-3f).count();

					if (num_active == 0)
					{
						manipulator.Controller()->_autd.AppendGainSync(autd::NullGain::Create());
					}
					else {
						std::vector<autd::GainPtr> gain_list(num_active);
						int id_begin_search = 0;
						for (auto itr_list = gain_list.begin(); itr_list != gain_list.end(); itr_list++) {
							std::map<int, autd::GainPtr> gain_map;
							auto itr_duties = std::find_if(dutiesStl.begin() + id_begin_search, dutiesStl.end(), [](float u) {return u > 0; });
							id_begin_search = std::distance(dutiesStl.begin(), itr_duties) + 1;
							for (int i_autd = 0; i_autd < num_autd; i_autd++) {
								if (i_autd == std::distance(dutiesStl.begin(), itr_duties)) {
									int amplitude = std::max(0, (std::min(255, static_cast<int>((*itr_duties) * 255.f * num_active))));
									//std::cout << amplitude << std::endl;
									gain_map.insert(std::make_pair(i_autd, autd::FocalPointGain::Create(posObserved, amplitude)));
								}
								else {
									gain_map.insert(std::make_pair(i_autd, autd::NullGain::Create()));
								}
							}
							*itr_list = autd::GroupedGain::Create(gain_map);
						}
						manipulator.Controller()->_autd.ResetLateralGain();
						manipulator.Controller()->_autd.AppendLateralGain(gain_list);
						manipulator.Controller()->_autd.StartLateralModulation(_freq);
					}
					Eigen::MatrixXf centersAutd = manipulator.Controller()->CentersAUTD();
					Eigen::Vector3f forceResult = manipulator.Controller()->arfModelPtr->arf(
						posObserved.replicate(1, manipulator.Controller()->_autd.geometry()->numDevices()) - centersAutd,
						manipulator.Controller()->eulerAnglesAUTD)
						* duties;
					ofs << observationTime << ", " << posObserved.x() << ", " << posObserved.y() << ", " << posObserved.z() << ", "
						<< vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
						<< integ.x() << ", " << integ.y() << ", " << integ.z() << ", "
						<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", "
						<< velTgt.x() << ", " << velTgt.y() << ", " << velTgt.z() << ", "
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

class linear_scan {
	Eigen::Vector3f _direction;
	Eigen::Vector3f _pos_start;
	float _dist_step;
	int _num_step;
	int _wait_seconds;
public:
	linear_scan(const Eigen::Vector3f& direction,
		const Eigen::Vector3f& pos_start = Eigen::Vector3f(0, 0, 0),
		float dist_step = 50.f,
		int num_step = 10,
		int wait_seconds = 60)
		:_direction(direction),
		_pos_start(pos_start),
		_dist_step(dist_step),
		_num_step(num_step),
		_wait_seconds(wait_seconds) {}

	void run(dynaman::FloatingObjectPtr objPtr) {
		for (int i_step = 0; i_step < _num_step; i_step++) {
			Eigen::Vector3f pos_tgt = _pos_start + i_step * _dist_step * _direction;
			std::cout << "updating target position: " << pos_tgt.transpose() << std::endl;
			objPtr->updateStatesTarget(pos_tgt, Eigen::Vector3f(0, 0, 0));
			std::this_thread::sleep_for(std::chrono::seconds(_wait_seconds));
		}
	}
};

class linear_maneuver_rep{
	Eigen::Vector3f _pos_start;
	Eigen::Vector3f _pos_end;
	float _v_init;
	float _delta_v;
	int _num_step;

public:
	linear_maneuver_rep(const Eigen::Vector3f& pos_start, const Eigen::Vector3f& pos_end, float v_init = 200, float delta_v = 100, int num_step = 20)
		:_pos_start(pos_start), _pos_end(pos_end), _v_init(v_init), _delta_v(delta_v), _num_step(num_step){}

	void run(dynaman::FloatingObjectPtr objPtr) {
		float time_init = 4 * (_pos_end - _pos_start).norm() / _v_init;
		std::this_thread::sleep_for(std::chrono::seconds(20));
		objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(10, timeGetTime() / 1000.f, objPtr->getPosition(), _pos_start));
		for (int i_step = 0; i_step < _num_step; i_step++) {
			float v_max = _v_init + i_step * _delta_v;
			float time_trans = _v_init / v_max * time_init;
			std::cout << "target velocity: " << v_max << "[mm/s]" << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(20));
			objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(time_trans, timeGetTime() / 1000.f, _pos_start, _pos_end));
			std::this_thread::sleep_for(std::chrono::seconds(20));
			objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(time_trans, timeGetTime() / 1000.f, _pos_end, _pos_start));
		}
	}
};

class circular_maneuver {
	Eigen::Vector3f _center;
	float _radius;
	float _raan;
	float _inclination;
	float _vel_init;
	float _vel_delta;
	float _phase_init;
	int _num_step;
public:

	circular_maneuver(float radius, 
		const Eigen::Vector3f& center = Eigen::Vector3f(0, 0, 0),
		float inclination = pi / 2.f,
		float raan = 0.f,
		float vel_init = 50.f,
		float vel_delta = 25.f,
		int num_step = 81,
		float phase_init = pi/2.f)
		:_center(center),
		_radius(radius),
		_inclination(inclination),
		_raan(raan),
		_vel_init(vel_init),
		_vel_delta(vel_delta),
		_num_step(num_step),
		_phase_init(phase_init){}

	void run(dynaman::FloatingObjectPtr objPtr) {
		std::this_thread::sleep_for(std::chrono::seconds(20));
		for (int i_step = 0; i_step < _num_step; i_step++) {
			float vel = _vel_init + i_step * _vel_delta;
			float period = 2.0f * _radius * pi / vel;
			std::cout << "target velocity: " << vel << "[mm/s] (period " << period << "[s])" << std::endl;
			objPtr->SetTrajectory(
				dynaman::TrajectoryCircle::Create(
					_center,
					_radius,
					_inclination,
					_raan,
					period,
					0.f,
					timeGetTime() / 1000.f
				)
			);
			std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(period * 1000)));
		}
	}
};

class recorder {
	std::thread th_rec;
	bool recording;
public:
	recorder():recording(false) {}

	void start_recording(const std::string& filename, dynaman::FloatingObjectPtr objPtr, int wait_ms = 30) {
		this->th_rec = std::thread(
			[this, &filename, objPtr, &wait_ms]() {
				std::ofstream ofs(filename);
				recording = true;
				ofs << "t, x, y, z, vx, vy, vz, xTgt, yTgt, zTgt, vxTgt, vyTgt, vzTgt, axTgt, ayTgt, azTgt" << std::endl;
				while (recording) {
					int time = timeGetTime();
					Eigen::Vector3f pos = objPtr->getPosition();
					Eigen::Vector3f vel = objPtr->getVelocity();
					Eigen::Vector3f posTgt = objPtr->getPositionTarget();
					Eigen::Vector3f velTgt = objPtr->getVelocityTarget();
					Eigen::Vector3f accelTgt = objPtr->getAccelTarget();
					ofs << time << ", "
						<< pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
						<< vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
						<< posTgt.x() << ", " << posTgt.y() << ", " << posTgt.z() << ", "
						<< velTgt.x() << ", " << velTgt.y() << ", " << velTgt.z() << ", "
						<< accelTgt.x() << ", " << accelTgt.y() << ", " << accelTgt.z() << ", "
						<< std::endl;
					std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
				}
				ofs.close();
			}
		);
	}

	void stop_recording() {
		recording = false;
		th_rec.join();
	}

	~recorder() {
		if (recording) {
			stop_recording();
		}
	}
};

cv::QtFont createFont(int error) {
	std::string fontname("Times");
	int pointsize = 40;
	auto color = (abs(error) < 50 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
	return cv::fontQt("Times", pointsize, color);
}

cv::Scalar createColor(int error) {
	return (abs(error) < 50 ? cv::Scalar(0, 155, 0) : cv::Scalar(0, 0, 155));
}

class ui {
public:
	enum direction {
		Defalt = 0,
		PX = 1,
		MX = 2,
		PY = 3,
		MY = 4,
		PZ = 5,
		MZ = 6
	};

	enum order {
		Grab = 0,
		Move = 1,
		Release = 2,
		Wait = 3
	};

private:
	bool running;
	std::thread t;
	int _wait_ms;
	direction _direction;
	order _order;
public:

	ui(int wait_ms = 30) :running(false), _wait_ms(wait_ms), _direction(direction::Defalt),_order(order::Wait){}

	void start(dynaman::FloatingObjectPtr objPtr) {
		t = std::thread(
			[this, objPtr]() {
				std::string winname("UI");
				int sidescreen_width, sidescreen_height, sidescreen_posx, sidescreen_posy;
				MonitorInfoManager::get_side_monitor_info(sidescreen_width, sidescreen_height, sidescreen_posx, sidescreen_posy);
				cv::Mat canvas(sidescreen_height, sidescreen_width, CV_8UC3, cv::Scalar::all(255));
				DrawUtil::imshowPopUp(winname.c_str(), canvas, sidescreen_posx, sidescreen_posy);
				cv::waitKey(3);
				cv::Point p_text(sidescreen_width * 0.1, sidescreen_height * 0.2);
				cv::Point p_x(sidescreen_width * 0.2, sidescreen_height * 0.4);
				cv::Point p_y(sidescreen_width * 0.2, sidescreen_height * 0.6);
				cv::Point p_z(sidescreen_width * 0.2, sidescreen_height * 0.8);
				running = true;
				int fontsize = 5;
				while (running) {
					cv::Mat img;
					canvas.copyTo(img);
					Eigen::Vector3f pos = objPtr->getPosition();
					Eigen::Vector3f posTgt;
					std::string instruction;
					std::string dest;
					switch (_direction)
					{
					case direction::Defalt:
						posTgt << 0, 0, 0; dest = "O (Center)"; break;
					case direction::PX:
						posTgt << 200, 0, 0; dest = "+X (Left)"; break;
					case direction::MX:
						posTgt << -200, 0, 0; dest = "-X (Right)"; break;
					case direction::PY:
						posTgt << 0, 100, 0 ; dest = "+Y (Near)"; break;
					case direction::MY:
						posTgt << 0, -100, 0; dest = "-Y (Far)"; break;
					case direction::PZ:
						posTgt << 0, 0, 200; dest = "+Z (High)"; break;
					case direction::MZ:
						posTgt << 0, 0, -200; dest = "-Z (Low)"; break;

					default:
						break;
					}
					switch (_order) {
					case ui::order::Grab:
						instruction = "Grab"; break;
					case ui::Move:
						instruction = "Move to " + dest; break;
					case ui::Release:
						instruction = "Release"; break;
						break;
					default:
						break;
					}
					int dx = pos.x() - posTgt.x(); int dy = pos.y() - posTgt.y(); int dz = pos.z() - posTgt.z();
					
					cv::putText(img, instruction, p_text, cv::FONT_HERSHEY_SIMPLEX, fontsize, cv::Scalar::all(0), 4);
					cv::putText(img, std::string("dx: ") + std::to_string(dx), p_x, cv::FONT_HERSHEY_SIMPLEX, fontsize, createColor(dx), 4);
					cv::putText(img, std::string("dy: ") + std::to_string(dy), p_y, cv::FONT_HERSHEY_SIMPLEX, fontsize, createColor(dy), 4);
					cv::putText(img, std::string("dz: ") + std::to_string(dz), p_z, cv::FONT_HERSHEY_SIMPLEX, fontsize, createColor(dz), 4);
					cv::imshow(winname, img);
					auto key = cv::waitKey(_wait_ms);
					if (key == 'q') {
						break;
					}
				}
			}
		);
	}

	void stop() {
		running = false;
		t.join();
	}

	void set_instruction(order order) {
		_order = order;
	}
	void set_destination(direction dest) {
		_direction = dest;
	}

	~ui() {
		if (running)
			stop();
	}
};

bool check_stable(dynaman::FloatingObjectPtr objPtr, const Eigen::Vector3f& posTgt, int duration_ms, int max_duration_ms) {
	int stable_time = 0;
	int total_time = 0;
	const int loop_time = 50;
	while (stable_time < duration_ms) {
		std::this_thread::sleep_for(std::chrono::milliseconds(loop_time));
		Eigen::Vector3f pos = objPtr->getPosition();
		if (abs(pos.x() - posTgt.x()) < 50 && abs(pos.y() - posTgt.y()) < 50 && abs(pos.z() - posTgt.z()) < 50) {
			stable_time += loop_time;
		}
		else {
			stable_time = 0;
		}
		total_time += loop_time;
		if (stable_time > duration_ms) {
			return true;
		}
		else if (total_time > max_duration_ms) {
			return false;
		}
	}
}

Eigen::Vector3f destination(ui::direction d) {
	Eigen::Vector3f pos;
	switch (d)
	{
	case ui::Defalt:
		pos << 0, 0, 0;
		break;
	case ui::PX:
		pos << 200, 0, 0;
		break;
	case ui::MX:
		pos << -200, 0, 0;
		break;
	case ui::PY:
		pos << 0, 100, 0;
		break;
	case ui::MY:
		pos << 0, -100, 0;
		break;
	case ui::PZ:
		pos << 0, 0, 200;
		break;
	case ui::MZ:
		pos << 0, 0, -200;
		break;
	default:
		pos << 0, 0, 0;
		break;
	}
	return pos;
}

Eigen::Vector3f vibration_direction(ui::direction d) {
	Eigen::Vector3f v;
	switch (d)
	{
	case ui::Defalt:
		v << 0, 0, 0;
		break;
	case ui::PX:
		v << 200, 0, 0;
		break;
	case ui::MX:
		v << -200, 0, 0;
		break;
	case ui::PY:
		v << 0, 100, 0;
		break;
	case ui::MY:
		v << 0, -100, 0;
		break;
	case ui::PZ:
		v << 0, 0, 200;
		break;
	case ui::MZ:
		v << 0, 0, -200;
		break;
	default:
		v << 0, 0, 0;
		break;
	}
	return v;
}

Eigen::Vector3f vibration_dest(ui::direction d) {
	Eigen::Vector3f dest;
	switch (d)
	{
	case ui::Defalt:
		dest << 0, 0, -200;
		break;
	case ui::PX:
		dest << 0, 0, -200;
		break;
	case ui::MX:
		dest << 0, 0, -200;
		break;
	case ui::PY:
		dest << 0, 0, -200;
		break;
	case ui::MY:
		dest << 0, 0, -200;
		break;
	case ui::PZ:
		dest << -200, 0, 0;
		break;
	case ui::MZ:
		dest << -200, 0, 0;
		break;
	default:
		dest << 0, 0, 0;
		break;
	}
	return dest;
}

ui::direction vibration_dest_direction(ui::direction d) {
	ui::direction dest;
	switch (d)
	{
	case ui::Defalt:
		dest = ui::direction::Defalt;
		break;
	case ui::PX:
		dest = ui::direction::MZ;
		break;
	case ui::MX:
		dest = ui::direction::MZ;
		break;
	case ui::PY:
		dest = ui::direction::MZ;
		break;
	case ui::MY:
		dest = ui::direction::MZ;
		break;
	case ui::PZ:
		dest = ui::direction::MX;
		break;
	case ui::MZ:
		dest = ui::direction::MX;
		break;
	default:
		dest = ui::direction::MZ;
		break;
	}
	return dest;
}

int main(int argc, char** argv) {

	std::cout << "Enter the name of applicant: ";
	std::string username;
	std::getline(std::cin, username);
	std::cout << "Enter the Experiment no:";
	std::string experiment_no;
	std::getline(std::cin, experiment_no);
	int exno = atoi(experiment_no.c_str());
	std::string filename = std::string("20200503_") + username + "_ex" + experiment_no;
	//std::string target_image_name("blue_target_cover.png");
	std::string target_image_name("blue_target_wo_cover.png");
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
	Eigen::Vector3f gainP = 10* Eigen::Vector3f::Constant(-1.6f);
	Eigen::Vector3f gainD = 5* Eigen::Vector3f::Constant(-4.0f);
	Eigen::Vector3f gainI = 5* Eigen::Vector3f::Constant(-0.05f);
	Eigen::Vector3f pos_init(0, 0, 0);
	manipulator.ocsPtr->SetGain(gainP, gainD, gainI);
	auto objPtr = dynaman::FloatingObject::Create(
		pos_init,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		-0.038e-3f,
		50.f);
	std::cout << "workspace " << std::endl
		<< "lower bound: " << objPtr->lowerbound().transpose() << std::endl
		<< "upper bound: " << objPtr->upperbound().transpose() << std::endl;

	int duration = 180000;//1800000; //30min
	int loopPeriod = 10;
	float focusBlur = 1;
	int freq = 100;
	float lambda = 0;

	//dynaman::balance_control_strategy strategy(gainP, gainD, gainI, loopPeriod, filename, focusBlur);
	//dynaman::simple_strategy strategy(gainP, gainD, gainI, loopPeriod, filename, focusBlur);
	dynaman::multiplex_strategy strategy(gainP, gainD, gainI, loopPeriod, filename, freq, lambda);
	std::thread th_control([&manipulator, &strategy, &objPtr, &duration]() {
		strategy.run(manipulator, objPtr, duration);
		}
	);

	float amplitude = 250;
	Eigen::Vector3f direction(1, 0, 0);
	float period = 12.0f;
	objPtr->SetTrajectory(dynaman::TrajectorySinusoid::Create(direction, amplitude, period, Eigen::Vector3f(0, 0, 0), timeGetTime() / 1000.f));

	th_control.join();
	manipulator.Close();

	return 0;
	//Eigen::Vector3f pos_start(0, -100, -100);
	//Eigen::Vector3f pos_end(0, 100, 100);
	//linear_maneuver_rep man(pos_start, pos_end, 300.f);
	//float radius_orbit = 100.f;
	//circular_maneuver man(radius_orbit, Eigen::Vector3f(0, 0, 0), pi/2.f, 0.f, 100.f, 0.f);
	//man.run(objPtr);
	/*Eigen::Vector3f direction(0, 0, 1);
	float amplitude = 200;
	float period = 5.0f;
	objPtr->SetTrajectory(dynaman::TrajectorySinusoid::Create(direction, amplitude, period, Eigen::Vector3f(0, 0, 0), timeGetTime() / 1000.f + 30.f));*/

	//std::this_thread::sleep_for(std::chrono::seconds(5));
	//recorder rec;
	//rec.start_recording("file1.csv", objPtr);
	//std::this_thread::sleep_for(std::chrono::seconds(10));
	//rec.stop_recording();
	//std::this_thread::sleep_for(std::chrono::seconds(5));
	//rec.start_recording("file2.csv", objPtr);
	//std::this_thread::sleep_for(std::chrono::seconds(20));
	//rec.stop_recording();

	std::vector<ui::direction> destinations(30);
	for (int i = 0; i < 5; i++) {
		destinations[i] = ui::direction::PX;
		destinations[i + 5] = ui::direction::MX;
		destinations[i + 10] = ui::direction::PY;
		destinations[i + 15] = ui::direction::MY;
		destinations[i + 20] = ui::direction::PZ;
		destinations[i + 25] = ui::direction::MZ;
	}

	std::cout << std::endl;
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());
	std::shuffle(destinations.begin(), destinations.end(), engine);
	std::cout << "destinations: ";
	std::for_each(destinations.begin(), destinations.end(),
		[](ui::direction d) {
			std::cout << d << ", ";
		}
	);

	ui display;
	recorder rec;
	std::ofstream ofs_success(filename + "_success.csv");
	int count = 0;
	switch (exno) {
	case 1:
		ofs_success << "count, destination, grab, release, release_stability, tGrabInit, tGrabFinished, timeMoveFinished, timeReleased" << std::endl;
		display.start(objPtr);
		for (auto itr_dest = destinations.begin(); itr_dest != destinations.end(); itr_dest++) {
			display.set_instruction(ui::order::Wait);
			display.set_destination(ui::direction::Defalt);
			std::string fname = username + "_ex" + experiment_no + "_count" + std::to_string(count) + "_to" + std::to_string(*itr_dest);
			objPtr->updateStatesTarget(destination(ui::direction::Defalt));
			std::cout << "stabilizing... (before grab)" << std::endl;
			if(count == 0)
				bool initStabilized = check_stable(objPtr, destination(ui::direction::Defalt), 3000, 10000);
			std::cout << "Press any key to start ." << std::endl;
			std::getline(std::cin, std::string());//getchar();
			//rec.start_recording(fname + "_grab.csv", objPtr, 33);
			auto timeGrabInit = timeGetTime();
			display.set_instruction(ui::order::Grab);
			std::cout << "Press a key after grabbed [Success: y, Failed: n]" << std::endl;

			std::string res_grab;
			std::getline(std::cin, res_grab);
			auto timeGrabFinished = timeGetTime();

			//std::cout << "stop recording ...";
			//rec.stop_recording();
			//rec.start_recording(fname + "_move.csv", objPtr, 33);
			display.set_destination(*itr_dest);
			display.set_instruction(ui::order::Move);
			check_stable(objPtr, destination(*itr_dest), 3000, 60000);
			auto timeMoveFinished = timeGetTime();
			//rec.stop_recording();
			std::this_thread::sleep_for(std::chrono::seconds(1));
			objPtr->resetIntegral();
			objPtr->updateStatesTarget(destination(*itr_dest));
			//rec.start_recording(fname + "_release.csv", objPtr, 33);
			display.set_instruction(ui::order::Release);
			std::cout << "Press a key after released [Success: y, Failed: n]." << std::endl;

			std::string res_release;
			std::getline(std::cin, res_release);
			auto timeReleased = timeGetTime();
			std::cout << "stabilizing... (after release)" << std::endl;
			bool releaseStability = check_stable(objPtr, destination(*itr_dest), 5000, 10000);
			//rec.stop_recording();
			ofs_success << std::distance(destinations.begin(), itr_dest) << ", "
				<< std::to_string(*itr_dest) << ", "
				<< res_grab << ", "
				<< res_release << ", "
				<< (releaseStability ? "True" : "False") << ", "
				<< timeGrabInit << ", "
				<< timeGrabFinished << ", "
				<< timeMoveFinished << ", "
				<< timeReleased
				<< std::endl;
		}
		display.stop();
		ofs_success.close();
		break;
	case 2:
		ofs_success << "count, destination, grab, release, release_stability, tGrabInit, tGrabFinished, timeMoveFinished, timeReleased" << std::endl;
		display.start(objPtr);
		for (auto itr_dest = destinations.begin(); itr_dest != destinations.end(); itr_dest++) {
			display.set_instruction(ui::order::Wait);
			display.set_destination(ui::direction::Defalt);
			objPtr->SetTrajectory(dynaman::TrajectoryBangBang::Create(6.0f, timeGetTime()/1000.f,destination(ui::direction::Defalt),destination(*itr_dest)));
			objPtr->updateStatesTarget(destination(*itr_dest));
			std::cout << "stabilizing... (before grab)" << std::endl;
			bool initStabilized = check_stable(objPtr, destination(*itr_dest), 3000, 10000);
			std::cout << "Press any key to start ." << std::endl;
			std::getline(std::cin, std::string());;
			//rec.start_recording(fname + "_grab.csv", objPtr, 33);
			auto timeGrabInit = timeGetTime();
			display.set_instruction(ui::order::Grab);
			std::cout << "Press a key after grabbed [Success: y, Failed: n]" << std::endl;

			std::string res_grab;
			std::getline(std::cin, res_grab);
			auto timeGrabFinished = timeGetTime();

			display.set_destination(ui::direction::Defalt);
			display.set_instruction(ui::order::Move);
			check_stable(objPtr, destination(ui::direction::Defalt), 3000, 60000);
			auto timeMoveFinished = timeGetTime();
			objPtr->resetIntegral();
			objPtr->updateStatesTarget(destination(ui::direction::Defalt));
			//rec.start_recording(fname + "_release.csv", objPtr, 33);
			display.set_instruction(ui::order::Release);
			std::cout << "Press a key after released [Success: y, Failed: n]." << std::endl;
			
			std::string res_release;
			std::getline(std::cin, res_release);
			auto timeReleased = timeGetTime();
			std::cout << "stabilizing... (after release)" << std::endl;
			bool releaseStability = check_stable(objPtr, destination(ui::direction::Defalt), 5000, 10000);
			ofs_success << std::distance(destinations.begin(), itr_dest) << ", "
				<< std::to_string(*itr_dest) << ", "
				<< res_grab << ", "
				<< res_release << ", "
				<< (releaseStability ? "True" : "False") << ", "
				<< timeGrabInit << ", "
				<< timeGrabFinished << ", "
				<< timeMoveFinished << ", "
				<< timeReleased
				<< std::endl;
		}
		display.stop();
		ofs_success.close();
		break;
	case 3:
		ofs_success << "count, destination, grab, release, release_stability, tGrabInit, tGrabFinished, timeMoveFinished, timeReleased" << std::endl;
		display.start(objPtr);
		for (auto itr_dest = destinations.begin(); itr_dest != destinations.end(); itr_dest++) {
			display.set_instruction(ui::order::Wait);
			display.set_destination(vibration_dest_direction(*itr_dest));
			objPtr->updateStatesTarget(destination(ui::direction::Defalt));
			std::cout << "stabilizing... (before grab)" << std::endl;
			bool initStabilized = check_stable(objPtr, destination(ui::direction::Defalt), 3000, 10000);
			std::cout << "Press any key to start ." << std::endl;
			std::getline(std::cin, std::string());
			//rec.start_recording(fname + "_grab.csv", objPtr, 33);
			auto timeGrabInit = timeGetTime();
			Eigen::Vector3f direction = vibration_direction(*itr_dest);
			float amplitude = 1;
			float period = 5.0f;
			objPtr->SetTrajectory(dynaman::TrajectorySinusoid::Create(direction, amplitude, period, Eigen::Vector3f(0, 0, 0), timeGetTime() / 1000.f));
			display.set_instruction(ui::order::Grab);
			std::cout << "Press a key after grabbed [Success: y, Failed: n]" << std::endl;

			std::string res_grab;
			std::getline(std::cin, res_grab);
			auto timeGrabFinished = timeGetTime();

			display.set_destination(vibration_dest_direction(*itr_dest));
			display.set_instruction(ui::order::Move);
			check_stable(objPtr, vibration_dest(*itr_dest), 3000, 60000);
			auto timeMoveFinished = timeGetTime();
			objPtr->resetIntegral();
			objPtr->updateStatesTarget(destination(ui::direction::Defalt));
			//rec.start_recording(fname + "_release.csv", objPtr, 33);
			display.set_instruction(ui::order::Release);
			std::cout << "Press a key after released [Success: y, Failed: n]." << std::endl;
			
			std::string res_release;
			std::getline(std::cin, res_release);
			auto timeReleased = timeGetTime();
			std::cout << "stabilizing... (after release)" << std::endl;
			bool releaseStability = check_stable(objPtr, destination(ui::direction::Defalt), 5000, 10000);
			ofs_success << std::distance(destinations.begin(), itr_dest) << ", "
				<< std::to_string(*itr_dest) << ", "
				<< res_grab << ", "
				<< res_release << ", "
				<< (releaseStability ? "True" : "False") << ", "
				<< timeGrabInit << ", "
				<< timeGrabFinished << ", "
				<< timeMoveFinished << ", "
				<< timeReleased
				<< std::endl;
		}
		display.stop();
		ofs_success.close();

		break;
	default:
		std::cout << "invalid experiment no." << std::endl;
		break;
	}

	th_control.join();
	manipulator.Close();

	return 0;
}