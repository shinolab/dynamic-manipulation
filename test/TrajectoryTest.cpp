#include "odcs.hpp"
#include <string>
#include <fstream>
#include <thread>

#pragma comment (lib, "winmm")

int main(int argc, char** argv) {
	std::string trajName("circular_4");
	Eigen::Vector3f center(100, 100, 100);
	float radius = 100;
	float inclination = M_PI/4.0f;
	float raan = M_PI/4.0f;
	float period = 10;
	float phaseInit = M_PI/2;
	float timeInit = timeGetTime() / 1000.f;
	std::ofstream ofs_config(trajName + "_config.txt");
	ofs_config << "center: " << center.transpose() << std::endl
		<< "radius: " << radius << std::endl
		<< "inclination: " << inclination << std::endl
		<< "raan: " << raan << std::endl
		<< "period: " << period << std::endl
		<< "phaseInit: " << phaseInit << std::endl
		<< "timeInit: " << timeInit << std::endl;
	ofs_config.close();
	std::ofstream ofs_log(trajName + "_log.csv");
	auto traj = dynaman::TrajectoryCircle::Create(
		center,
		radius,
		inclination,
		raan,
		period,
		phaseInit,
		timeInit
	);
	int timeStart = timeGetTime();
	ofs_log << "t, x, y, z, vx, vy, vz, ax, ay, az" << std::endl;
	while (timeGetTime() - timeStart < 20000) {
		float currentTime = timeGetTime()/1000.f;
		Eigen::Vector3f pos = traj->pos(currentTime);
		Eigen::Vector3f vel = traj->vel(currentTime);
		Eigen::Vector3f accel = traj->accel(currentTime);
		ofs_log << currentTime - timeInit << ", "
			<< pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
			<< vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
			<< accel.x() << ", " << accel.y() << ", " << accel.z() << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	ofs_log.close();
	return 0;

}