#include <ctime>
#include <fstream>
#include <future>
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
constexpr float NUM_ERROR_COND = 50;
constexpr float POS_ERROR_MAX = 250;


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
	Eigen::Vector3f posInitTgt = 200 * Eigen::Vector3f(-1, 0, 0).normalized();
	Eigen::Vector3f posEndTgt = 200 * Eigen::Vector3f(1, 0, 0).normalized();

	// ***********************************
	char date[sizeof("yymmdd_HHMMSS")];
	auto tt = std::time(nullptr);
	std::strftime(&date[0], sizeof(date), "%y%m%d_%H%M%S", std::localtime(&tt));
	std::string prefix(date);
	std::string filename = prefix + "_v" + std::string(argv[1]) + "_basin_summery_th.csv";
	std::string filenameLog = prefix + "_v" + std::string(argv[1]) + "_log_t.txt";
	std::ofstream ofs(filename);
	std::ofstream ofsLog(filenameLog);

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


	std::vector<Eigen::Vector3f> velErrors;
	std::vector<Eigen::Vector3f> posErrors;
	//addRandomUnitVectors(velErrors, NUM_ERROR_COND);
	//addRandomUnitVectors(posErrors, NUM_ERROR_COND);
	addUnitVectorsFib(velErrors, NUM_ERROR_COND);
	addUnitVectorsFib(posErrors, NUM_ERROR_COND);

	ofs << posInitTgt.transpose() << std::endl;
	ofs << posEndTgt.transpose() << std::endl;
	ofs << "velv";
	for (auto&& veLv : velErrorLevels) {
		ofs << "," << veLv;
	}
	ofs << std::endl;

	for (auto&& veLv : velErrorLevels) {
		auto start = std::chrono::system_clock::now();

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
				std::vector<std::future<void>> fut;
				std::vector<bool> isConverged(velErrors.size());
				for (int idv = 0; idv < velErrors.size(); idv++) {
					fut.push_back(
						std::async(
							[&isConverged, idv, veLv, &timeTransStart, &timeToTrans, &velErrors, &posErrorMid, &pd, &pAupa, &pTracker, &pTrajectory]()
							{
								auto pe = posErrorMid * pd;
								auto ve = veLv * velErrors[idv];
								const float timeInit = timeTransStart + timeToTrans;
								const float timeEnd = timeInit + 10.0f;
								isConverged[idv] = simulate(pAupa, pTracker, pTrajectory, pe, ve, timeInit, timeEnd);
							}
						)
					);
				}
				for (auto&& f : fut) {
					f.get();
				}
				if (std::find(isConverged.begin(), isConverged.end(), false) != isConverged.end()) {
					convergedInAllTrials = false;
					break;
				}
				if (!convergedInAllTrials)
					break;
			}
			convergedInAllTrials ? posErrorLb = posErrorMid : posErrorUb = posErrorMid;
		}
		ofs << "," << posErrorMid;
		std::cout << "vLv: " << veLv << " mm/s finished." << std::endl;
		auto end = std::chrono::system_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		ofsLog << "elapsed:" << elapsed.count() << "ms" << std::endl;
		std::cout << "elapsed:" << elapsed.count() << "ms" << std::endl;
	}
	ofs.close();
	ofsLog.close();

	return 0;
}