#include "odcs.hpp"
#include <string>
#include <fstream>
#include <thread>

#pragma comment (lib, "winmm")

int main(int argc, char** argv) {
	std::string trajName("sinusoid_4");
	Eigen::Vector3f center(0, 0, 0);
	Eigen::Vector3f direction = Eigen::Vector3f::UnitZ();
	float amplitude = 200;
	float period = 5.f;
	float timeInit = timeGetTime() / 1000.f;
	std::ofstream ofs_config(trajName + "_config.txt");
	ofs_config << "center: " << center.transpose() << std::endl
		<< "direction: " << direction << std::endl
		<< "period: " << period << std::endl
		<< "timeInit: " << timeInit << std::endl;
	ofs_config.close();
	std::ofstream ofs_log(trajName + "_log.csv");
	auto traj = dynaman::TrajectorySinusoid::Create(
		direction,
		amplitude,
		period,
		center,
		timeGetTime() / 1000.f
	);
	int timeStart = timeGetTime();
	ofs_log << "t,phase, x, y, z, vx, vy, vz, ax, ay, az" << std::endl;
	while (timeGetTime() - timeStart < 20000) {
		float currentTime = timeGetTime()/1000.f;
		float phase = traj->Phase(currentTime);
		Eigen::Vector3f pos = traj->pos(currentTime);
		Eigen::Vector3f vel = traj->vel(currentTime);
		Eigen::Vector3f accel = traj->accel(currentTime);
		ofs_log << currentTime - timeInit << ", "
			<< phase << ", "
			<< pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
			<< vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
			<< accel.x() << ", " << accel.y() << ", " << accel.z() << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	ofs_log.close();
	return 0;

}