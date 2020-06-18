#include "odcs.hpp"
#include <string>
#include <fstream>
#include <thread>

#pragma comment (lib, "winmm")

int main(int argc, char** argv) {
	std::string trajName("heartShape_3");
	Eigen::Vector3f center(0, 0, 0);
	float height = 1;
	float width = 1;
	float period = 2.0f;
	DWORD timeInit = timeGetTime();
	std::ofstream ofs_config(trajName + "_config.txt");
	ofs_config << "center: " << center.transpose() << std::endl
		<< "height: " << height << std::endl
		<< "width: " << width << std::endl
		<< "period: " << period << std::endl;
	ofs_config.close();
	std::ofstream ofs_log(trajName + "_log.csv");
	auto traj = dynaman::TrajectoryHeart::Create(
		center,
		height,
		width,
		period,
		timeInit
	);
	//auto traj = dynaman::TrajectoryInfShape::Create(
	//	center,
	//	height,
	//	width,
	//	period,
	//	timeInit
	//);
	DWORD timeStart = timeGetTime();
	DWORD timeBefore;
	Eigen::Vector3f posBefore = traj->pos(timeStart);
	Eigen::Vector3f velBefore = traj->vel(timeStart);
	ofs_log << "t, phase, x, y, z, vx, vy, vz, ax, ay, az"
		<< ", dxdt, dydt, dzdt, d(vx)dt, d(vy)dt, d(vt)dt" << std::endl;
	while (timeGetTime() - timeStart < 5000) {
		DWORD currentTime = timeGetTime();
		float phase = traj->Phase(currentTime);
		Eigen::Vector3f pos = traj->pos(currentTime);
		Eigen::Vector3f vel = traj->vel(currentTime);
		Eigen::Vector3f accel = traj->accel(currentTime);
		float dt = (currentTime - timeBefore) / 1000.f;
		Eigen::Vector3f drdt = (pos - posBefore) / dt;
		Eigen::Vector3f dvdt = (vel - velBefore) / dt;
		//std::cout << dt << std::endl;
		timeBefore = currentTime;
		posBefore = pos;
		velBefore = vel;
			ofs_log << currentTime - timeInit << ", "
			<< phase << ", "
			<< pos.x() << ", " << pos.y() << ", " << pos.z() << ", "
			<< vel.x() << ", " << vel.y() << ", " << vel.z() << ", "
			<< accel.x() << ", " << accel.y() << ", " << accel.z() << ", "
			<< drdt.x() << ", " << drdt.y() << ", " << drdt.z() << ", "
			<< dvdt.x() << ", " << dvdt.y() << ", " << dvdt.z() << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	ofs_log.close();
	return 0;

}