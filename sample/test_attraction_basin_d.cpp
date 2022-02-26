#include <ctime>
#include <fstream>
#include <string>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "GainPlan.hpp"
#include "manipulator.hpp"
#include "ThrustSearch.hpp"
#include "haptic_icon.hpp"
#include "dynaman_simulator.hpp"

using namespace dynaman;

constexpr float DUTY_FF_MAX = 0.5f;
constexpr int NUM_BINARY_SEARCH_MAX = 8;
constexpr int NUM_WAYPOINTS = 10;
constexpr float NUM_ERROR_COND = 50;
constexpr float POS_ERROR_MAX = 210;


int main(int argc, char** argv) {
	//Experiment Condition**********************
	std::vector<float> velErrorLevels;// { 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000 };
	for (int iArg = 1; iArg < argc; iArg++) {
		std::string str_argv(argv[iArg]);
		auto argvf = std::stof(str_argv);
		velErrorLevels.push_back(argvf);
	}
	std::cout << "velErrorLevels: ";
	for (auto&& v : velErrorLevels) {
		std::cout << ", " << v;
	}
	std::cout << std::endl;
	Eigen::Vector3f posInitTgt = 200 * Eigen::Vector3f(-1, 0, -1).normalized();
	Eigen::Vector3f posEndTgt = 200 * Eigen::Vector3f(1, 0, 1).normalized();

	// ***********************************
	char date[sizeof("yymmdd_HHMMSS")];
	auto tt = std::time(nullptr);
	std::strftime(&date[0], sizeof(date), "%y%m%d_%H%M%S", std::localtime(&tt));
	std::string prefix(date);
	std::string filename = prefix + "_v" + std::string(argv[1]) + "_basin_summery_d.csv";
	std::string filenameLog = prefix + "_v" + std::string(argv[1]) + "_log_d.txt";
	std::ofstream ofs(filename);
	std::ofstream ofsLog(filenameLog);

	ofs << "dv";
	for (int it = 0; it < NUM_WAYPOINTS; it++) {
		ofs << ",t=" << it;
	}
	ofs << std::endl;

	/*dependnet conditions*/
	const float timeTransStart = 0.0f;// simulation constant
	auto pAupa = haptic_icon::CreateController();
	auto pTracker = haptic_icon::CreateTracker("blue_target_r50mm.png");

	auto pTrajectory = CreateBangbangTrajecotryWithDrag(
		pAupa,
		posInitTgt,
		posEndTgt,
		RADIUS,
		timeTransStart,
		DUTY_FF_MAX
	);

	auto timeToTrans
		= dynamic_cast<dynaman::TrajectoryBangbangWithDrag*>(pTrajectory.get())->time_to_accel()
		+ dynamic_cast<dynaman::TrajectoryBangbangWithDrag*>(pTrajectory.get())->time_to_decel();

	auto start = std::chrono::system_clock::now();

	std::vector<Eigen::Vector3f> velErrors;
	std::vector<Eigen::Vector3f> posErrors;
	//addRandomUnitVectors(velErrors, NUM_ERROR_COND);
	//addRandomUnitVectors(posErrors, NUM_ERROR_COND);
	addUnitVectorsFib(velErrors, NUM_ERROR_COND);
	addUnitVectorsFib(posErrors, NUM_ERROR_COND);
	for (auto&& veLv : velErrorLevels) {
		ofs << veLv;

		for (int it = 0; it < NUM_WAYPOINTS; it++) {
			float posErrorLb = 0;
			float posErrorUb = POS_ERROR_MAX;
			float posErrorMid;

			for (int ib = 0; ib < NUM_BINARY_SEARCH_MAX; ib++) {
				auto convergedInAllTrials = true;
				posErrorMid = 0.5 * (posErrorLb + posErrorUb);
				if (posErrorUb - posErrorMid < 1 || posErrorMid - posErrorLb < 1) {
					break;
				}
				for (auto&& pd : posErrors) {
					for (auto&& vd : velErrors) {
						auto pe = posErrorMid * pd;
						auto ve = veLv * vd;
						const float timeInit = timeTransStart + it * timeToTrans / NUM_WAYPOINTS;
						const float timeEnd = timeInit + 10.0f;
						auto isConverged = simulate(pAupa, pTracker, pTrajectory, pe, ve, timeInit, timeEnd);
						//std::cout << "isConverged: " << std::boolalpha << isConverged << std::endl;
						if (!isConverged) {
							convergedInAllTrials = false;
							std::cout
								<< "failed to converge: (posError: " << pe.transpose()
								<< ", velError: " << ve.transpose() << ")"
								<< "(norm:" << pe.norm() << ", " << ve.norm() << ")" << std::endl;
							ofsLog
								<< "failed to converge: (posError: " << pe.transpose()
								<< ", velError: " << ve.transpose() << ")"
								<< "(norm:" << pe.norm() << ", " << ve.norm() << ")" << std::endl;
							break;
						}
					}
					if (!convergedInAllTrials)
						break;
				}
				for (int ic = 0; ic < NUM_ERROR_COND; ic++) {
				}
				if (convergedInAllTrials) {
					ofsLog << "converged in all trials: posErrorLv: " << posErrorMid << "mm @ v=" << veLv << "mm/s, waypoint: " << it << ")" << std::endl;
					std::cout << "converged in all trials: posErrorLv: " << posErrorMid << "mm @ v=" << veLv << "mm/s, waypoint: " << it << ")" << std::endl;
				}
				convergedInAllTrials ? posErrorLb = posErrorMid : posErrorUb = posErrorMid;
			}
			ofsLog << "Tolerant pos error is:" << posErrorMid << " mm @ v=" << veLv << "mm/s, waypoint: " << it << std::endl;
			std::cout << "Tolerant pos error is:" << posErrorMid << " mm @ v=" << veLv << "mm/s, waypoint: " << it << std::endl;
			ofs << "," << posErrorMid;
		}
		ofs << std::endl;
		auto end = std::chrono::system_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		ofsLog << "elapsed:" << elapsed.count() << "ms" << std::endl;
		std::cout << "elapsed:" << elapsed.count() << "ms" << std::endl;
	}
	ofs.close();
	ofsLog.close();

	return 0;
}