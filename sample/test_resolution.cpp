#include "dynaman_simulator.hpp"
#include "haptic_icon.hpp"

Eigen::Vector3f SCALE(200, 100, 200);

bool run_single_simulation(
	int steps_resoultion,
	const Eigen::Vector3f& posStart = SCALE.asDiagonal() * Eigen::Vector3f::Random(),
	const Eigen::Vector3f& posEnd = SCALE.asDiagonal() * Eigen::Vector3f::Random(),
	std::function<void(const state_type&, const float)> observer = [](const state_type&, const float) {}
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

	auto isConverged = simulate(
		pAupa,
		pTracker,
		pTrajectory,
		Eigen::Vector3f::Zero(),
		Eigen::Vector3f::Zero(),
		timeStart,
		timeStart + 10.0f,
		steps_resoultion,
		observer
	);

	return isConverged;
}

int main(int argc, char** argv) {


	std::string filename("20220222_fib_sphere_test.csv");
	std::vector<Eigen::Vector3f> vectors;
	addUnitVectorsFib(vectors, 120);
	std::ofstream ofs(filename);
	std::cout << vectors.size() << std::endl;
	for (auto&& v : vectors) {
		ofs << v.x() << "," << v.y() << "," << v.z() << "," << std::endl;
	}
	return 0;
	
	char date[sizeof("yymmdd_HHMMSS")];
	auto tt = std::time(nullptr);
	std::strftime(&date[0], sizeof(date), "%y%m%d_%H%M%S", std::localtime(&tt));
	
	auto start = std::chrono::system_clock::now();
	std::vector<int> steps_resolution{1, 3, 7, 15, 31, 127, 255};

	for (auto&& s : steps_resolution) {
		std::string prefix(date);
		std::string filename = prefix + "_resolution" + std::to_string(s) + "_test.csv";
		std::ofstream ofs(filename);
		run_single_simulation(
			s,
			200 * Eigen::Vector3f(1, 0, -1).normalized(),
			200 * Eigen::Vector3f(1, 0, 1).normalized(),
			[&ofs](const state_type& x, const float t)
			{
				ofs << t << "," << x[0] << "," << x[1] << "," << x[2] << "," << x[3] << "," << x[4] << "," << x[5] << std::endl;
			}
		);
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