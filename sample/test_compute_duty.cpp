#include <algorithm>
#include <ctime>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <random>
#include "GainPlan.hpp"
#include "haptic_icon.hpp"

using namespace dynaman;

//creates 2*n + 1 points that are equally distributed on a unit sphere.
std::vector<Eigen::Vector3f> pointsOnSphereFib(size_t n) {
	std::vector<Eigen::Vector3f> res;
	float tau = (std::sqrtf(5.0f) - 1.0f) / 2.0f;
	for (int i = -n; i < n + 1; i++) {
		auto theta = std::asinf(2 * i / (2 * n + 1));
		auto phi = 2 * pi * i * tau;
		res.emplace_back(
			std::cos(theta) * std::cos(phi),
			std::cos(theta) * std::sin(phi),
			std::sin(theta)
		);
	}
	return res;
}

int main(int argc, char** argv) {

	char date[sizeof("yymmdd_HHMMSS")];
	auto t = std::time(nullptr);
	std::strftime(&date[0], sizeof(date), "%y%m%d_%H%M%S", std::localtime(&t));
	std::string filename(date);
	filename += "_dutyTestLog.csv";
	
	std::ofstream ofs(filename);

	auto pAupa = std::make_shared<autd::Controller>();
	haptic_icon::SetGeometry(pAupa);
	auto pTracker = haptic_icon::CreateTracker("blue_target_r50mm.png");

	MultiplexManipulator manipulator(
		pAupa,
		pTracker
	);

	constexpr size_t NX = 2;
	constexpr size_t NY = 2;
	constexpr size_t NZ = 2;
	constexpr size_t NF = 500;
	constexpr float X_MIN = 0;
	constexpr float X_MAX = 0;
	constexpr float Y_MIN = 0;
	constexpr float Y_MAX = 0;
	constexpr float Z_MIN = 0;
	constexpr float Z_MAX = 0;
	constexpr float F_MAX = 1.5;

	ofs <<"x,y,z"
		<<",fx(tgt),fy(tgt),fz(tgt)"
		<<",fx(app),fy(app),fz(app)"
		<<",fx(heu),fy(heu),fz(heu)"
		<<",fx(opt),fy(opt),fz(opt)"
		<< std::endl;
	std::cout << "Error: (approx), (heuristics)" << std::endl;

	auto directions = pointsOnSphereFib(NF);

	for (int numAupa = 1; numAupa < 12; numAupa++) {

		for (int ix = 0; ix < NX; ix++) {
			for (int iy = 0; iy < NY; iy++) {
				for (int iz = 0; iz < NZ; iz++) {
					Eigen::Vector3f pos(
						X_MIN + (X_MAX - X_MIN) / (NX - 1) * ix,
						Y_MIN + (Y_MAX - Y_MIN) / (NY - 1) * iy,
						Z_MIN + (Z_MAX - Z_MIN) / (NZ - 1) * iz
					);

					for (auto&& direction : directions) {
						auto force = F_MAX * direction;
						//auto duty_approx = manipulator.ComputeDuty(force, pos);
						auto duty_heuristics = manipulator.ComputeDuty(force, pos, numAupa);
						//auto duty_opt = manipulator.ComputeDuty(force, pos, 12);
						//auto posRel = pos.replicate(1, pAupa->geometry()->numDevices()) - CentersAutd(pAupa->geometry());
						//auto f_mat = manipulator.arfModel()->arf(posRel, RotsAutd(pAupa->geometry()));
						//auto force_approx = f_mat * duty_approx;
						//auto force_heuristics = f_mat * duty_heuristics;
						//auto force_opt = f_mat * duty_opt;
						//ofs << pos.x() << "," << pos.y() << "," << pos.z()
						//	<< "," << force_approx.x() << "," << force_approx.y() << "," << force_approx.z()
						//	<< "," << force_heuristics.x() << "," << force_heuristics.y() << "," << force_heuristics.z()
						//	//<< "," << force_opt.x() << "," << force_opt.y() << "," << force_opt.z()
						//	<< std::endl;
						//std::cout << "Error: "
						//	<< std::fixed << std::setprecision(5)
						//	<< (force - force_approx).norm()
						//	<< "," << (force - force_heuristics).norm()
						//	//<< "," << (force - force_opt).norm()
						//	<< std::endl;
					}
				}
			}
		}
	}
	ofs.close();
	return 0;
}