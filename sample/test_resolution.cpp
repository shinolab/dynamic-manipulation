#include "dynaman_simulator.hpp"
#include "haptic_icon.hpp"

Eigen::Vector3f SCALE(200, 100, 200);
const float T_STATIONARY = 14.0f;

bool run_single_simulation(
	int steps_resoultion,
	const Eigen::Vector3f& posStart = SCALE.asDiagonal() * Eigen::Vector3f::Random(),
	const Eigen::Vector3f& posEnd = SCALE.asDiagonal() * Eigen::Vector3f::Random(),
	std::function<void(const state_type&, const float)> observer = [](const state_type&, const float) {},
	std::vector<float>& err_before = std::vector<float>(),
	std::vector<float>& err_after = std::vector<float>()
) {
	constexpr float DUTY_FF_MAX = 0.5f;
	
	std::cout << "posStart: (" << posStart.transpose()
		<< ") ---> posEnd: (" << posEnd.transpose() << ")" << std::endl;
	const float timeStart = 0.0f;// simulation constant
	auto pAupa = haptic_icon::CreateController();
	auto pTracker = haptic_icon::CreateTracker("blue_target_r50mm.png");

	auto pTrajectory = CreateBangbangTrajecotryWithDrag(
		pAupa,
		posStart,
		posEnd,
		RADIUS,
		timeStart,
		DUTY_FF_MAX
	);
	auto tTrans
		= dynamic_cast<TrajectoryBangbangWithDrag*>(pTrajectory.get())->time_to_accel()
		+ dynamic_cast<TrajectoryBangbangWithDrag*>(pTrajectory.get())->time_to_decel();

	//auto pTrajectory = TrajectoryStep::Create(
	//	posStart,
	//	posEnd,
	//	1
	//);

	auto isConverged = simulate(
		pAupa,
		pTracker,
		pTrajectory,
		Eigen::Vector3f::Zero(),
		Eigen::Vector3f::Zero(),
		timeStart,
		timeStart + tTrans + T_STATIONARY,
		steps_resoultion,
		[&observer, &err_before, &err_after, &tTrans, pTrajectory](const state_type& x, const float t) {
			observer(x, t);
			auto err = (Eigen::Vector3f(x[0], x[1], x[2]) - pTrajectory->pos(static_cast<DWORD>(1000 * t))).norm();
			t < tTrans ? err_before.push_back(DT_STEP * err / tTrans) : err_after.push_back(DT_STEP * err / T_STATIONARY); 
		}
	);
	return isConverged;
}

int main(int argc, char** argv) {
	
	char date[sizeof("yymmdd_HHMMSS")];
	auto tt = std::time(nullptr);
	std::strftime(&date[0], sizeof(date), "%y%m%d_%H%M%S", std::localtime(&tt));
	
	auto start = std::chrono::system_clock::now();

	constexpr int num_trials = 100;

	std::vector<int> steps_resolution;
	for (int r = 0; r < 256; r++) {
		steps_resolution.push_back(r);
	}
	std::string prefix(date);
	std::string filename = prefix + "_resolution_test_bb_v.csv";
	std::ofstream ofs(filename);
	ofs << "res,err(trans),err(stationary)" << std::endl;
	Eigen::Vector3f posStart = 200 * Eigen::Vector3f(1, 0, -1).normalized();
	Eigen::Vector3f posEnd = 200 * Eigen::Vector3f(1, 0, 1).normalized();
	for (auto&& s : steps_resolution) {
		std::vector<float> error_before;
		std::vector<float> error_after;
		run_single_simulation(
			s,
			posStart,
			posEnd,
			[](const state_type& x, const float t)
			{
//				ofs << t << "," << x[0] << "," << x[1] << "," << x[2] << "," << x[3] << "," << x[4] << "," << x[5] << std::endl;
			},
			error_before, error_after
		);
		float err_average_before = 0;
		for (auto itr = error_before.begin(); itr != error_before.end() - 1; itr++) {
			err_average_before += 0.5f * (*itr + *(itr + 1));
		}

		float err_average_after = 0;
		for (auto itr = error_after.begin(); itr != error_after.end() - 1; itr++) {
			err_average_after += 0.5f * (*itr + *(itr + 1));
		}
		std::cout << "res=" << s
			<< ", error (before): " << err_average_before
			<< ", (after) : " << err_average_after
			<< std::endl;
		ofs << s << "," << err_average_before << "," << err_average_after << std::endl;
		//constexpr int NUM_TRIAL = 100;
		//int count_success = 0;
		//for (int i = 0; i < NUM_TRIAL; i++) {
		//	if (run_single_simulation(s)) {
		//		count_success++;
		//	}
		//}
		//std::cout  << "success_rate: (" << count_success << " / " << NUM_TRIAL << ") for resolution steps:" << s << std::endl;
	}
	auto end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "elapsed: " << elapsed.count() << std::endl;
}