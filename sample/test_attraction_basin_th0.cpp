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
	Eigen::Vector3f posInitTgt = 200 * Eigen::Vector3f(-1, 0, 0).normalized();
	Eigen::Vector3f posEndTgt = 200 * Eigen::Vector3f(1, 0, 0).normalized();

	// ***********************************
	char date[sizeof("yymmdd_HHMMSS")];
	auto tt = std::time(nullptr);
	std::strftime(&date[0], sizeof(date), "%y%m%d_%H%M%S", std::localtime(&tt));
	std::string prefix(date);
	std::string filename = prefix + "_vh0_basin_summery_th.csv";
	std::string filenameLog = prefix + "_vh0_log_t.txt";
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


	std::vector<Eigen::Vector3f> posErrors;
	addUnitVectorsFib(posErrors, NUM_ERROR_COND);

	ofs << posInitTgt.transpose() << std::endl;
	ofs << posEndTgt.transpose() << std::endl;

	auto start = std::chrono::system_clock::now();

	float posErrorLb = 0;
	float posErrorUb = POS_ERROR_MAX;
	float posErrorMid;

	std::cout << "starting binary search..." << std::endl;
	for (int ib = 0; ib < NUM_BINARY_SEARCH_MAX; ib++) {
		auto convergedInAllTrials = true;
		posErrorMid = 0.5 * (posErrorLb + posErrorUb);
		if (posErrorUb - posErrorMid < 1 || posErrorMid - posErrorLb < 1) {
			break;
		}
		std::vector<std::future<void>> fut;
		std::vector<bool> isConverged(posErrors.size());

		for (int idr = 0; idr < posErrors.size(); idr++) {
			fut.push_back(
				std::async(
					[&isConverged, idr, &posErrors, &timeTransStart, &timeToTrans, &posErrorMid, &pAupa, &pTracker, &pTrajectory]()
					{
						auto pe = posErrorMid * posErrors[idr];
						auto ve = Eigen::Vector3f::Zero();
						const float timeInit = timeTransStart + timeToTrans;
						const float timeEnd = timeInit + 10.0f;
						isConverged[idr] = simulate(pAupa, pTracker, pTrajectory, pe, ve, timeInit, timeEnd);
					}
				)
			);
		}
		for (auto&& f : fut) {
			f.get();
		}
		auto itr_fail = std::find(isConverged.begin(), isConverged.end(), false);
		if (itr_fail != isConverged.end()) {
			convergedInAllTrials = false;
			auto pe = posErrorMid * posErrors[std::distance(isConverged.begin(), itr_fail)];
			std::cout << "Failed at posError: "
				<< pe.transpose()
				<< " (norm: " << pe.norm() << ")" << std::endl;
		}
		convergedInAllTrials ? posErrorLb = posErrorMid : posErrorUb = posErrorMid;
	}
	std::cout << "position tolerance:" << posErrorMid << std::endl;;
	auto end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	ofsLog << "elapsed:" << elapsed.count() << "ms" << std::endl;
	std::cout << "elapsed:" << elapsed.count() << "ms" << std::endl;
	
	ofs.close();
	ofsLog.close();

	return 0;
}