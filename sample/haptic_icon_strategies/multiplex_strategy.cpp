#include "multiplex_strategy.hpp"
#include "geometryUtil.hpp"
#include <fstream>
#include <string>
#define NOMINMAX
#include "Windows.h"

#pragma comment (lib, "winmm")

namespace dynaman {
	multiplex_strategy::multiplex_strategy(
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
		_lambda(lambda) {}

	int multiplex_strategy::run(dynaman::odcs& manipulator, FloatingObjectPtr objPtr, int duration) {
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
}