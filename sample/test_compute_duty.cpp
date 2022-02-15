#include <algorithm>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include "GainPlan.hpp"
#include "haptic_icon.hpp"

using namespace dynaman;

int main(int argc, char** argv) {

	char date[sizeof("yymmdd_HHMMSS")];
	auto t = std::time(nullptr);
	std::strftime(&date[0], sizeof(date), "%y%m%d_%H%M%S", std::localtime(&t));
	std::string filename(date);
	filename += "_dutyTestLog.csv";
	
	std::cout << filename << std::endl;
	float x = 0.1234567;
	std::cout << std::fixed << std::setprecision(5) << x << "," << x << "," << x << std::endl;

	return 0;
	std::ofstream ofs(filename);

	auto pAupa = std::make_shared<autd::Controller>();
	haptic_icon::SetGeometry(pAupa);
	auto pTracker = haptic_icon::CreateTracker("blue_target_r50mm.png");

	MultiplexManipulator manipulator(
		pAupa,
		pTracker
	);


	constexpr size_t NX = 6;
	constexpr size_t NY = 6;
	constexpr size_t NZ = 6;
	constexpr size_t NF = 10;
	constexpr float X_MIN = -200;
	constexpr float X_MAX = 200;
	constexpr float Y_MIN = -100;
	constexpr float Y_MAX = 100;
	constexpr float Z_MIN = -200;
	constexpr float Z_MAX = 200;
	constexpr float F_MAX = 10;

	ofs <<"x,y,z"
		<<",fx(tgt),fy(tgt),fz(tgt)"
		<<",fx(app),fy(app),fz(app)"
		<<",fx(heu),fy(heu),fz(heu)"
		<<",fx(opt),fy(opt),fz(opt)"
		<< std::endl;
	for (int ix = 0; ix < NX; ix++) {
		for (int iy = 0; iy < NY; iy++) {
			for (int iz = 0; iz < NZ; iz++) {
				Eigen::Vector3f pos(
					X_MIN + (X_MAX - X_MIN) / (NX - 1) * ix,
					Y_MIN + (Y_MAX - Y_MIN) / (NY - 1) * iy,
					Z_MIN + (Z_MAX - Z_MIN) / (NZ - 1) * iz
				);
				auto force = F_MAX * Eigen::Vector3f::Random().normalized();
				auto duty_approx = manipulator.ComputeDuty(force, pos);
				auto duty_heuristics = manipulator.ComputeDuty(force, pos, 4);
				auto duty_opt = manipulator.ComputeDuty(force, pos, 12);
				auto posRel = pos.replicate(1, pAupa->geometry()->numDevices()) - CentersAutd(pAupa->geometry());
				auto f_mat = manipulator.arfModel()->arf(posRel, RotsAutd(pAupa->geometry()));
				auto force_approx = f_mat * duty_approx;
				auto force_heuristics = f_mat * duty_heuristics;
				auto force_opt = f_mat * duty_opt;
				ofs << pos.x() << "," << pos.y() << "," << pos.z()
					<< "," << force_approx.x() << "," << force_approx.y() << "," << force_approx.z()
					<< "," << force_heuristics.x() << "," << force_heuristics.y() << "," << force_heuristics.z()
					<< "," << force_opt.x() << "," << force_opt.y() << "," << force_opt.z()
					<< std::endl;
				std::cout<< std::fixed << std::setprecision(5)
					<< (force - force_approx).norm()
					<< "," << (force - force_heuristics).norm()
					<< "," << (force - force_opt).norm()
					<< std::endl;
			}
		}
	}
	
	ofs.close();
	return 0;
}