#ifndef _DYNAMAN_SIMULATOR_HPP
#define _DYNAMAN_SIMULATOR_HPP

#include "manipulator.hpp"
#include "GainPlan.hpp"


using namespace dynaman;
using state_type = std::vector<float>;

constexpr auto DUTY_MIN = 1.0f / 255.0f;
constexpr float DT_STEP = 0.0005f;
constexpr float DT_OBS = 0.01f;
constexpr auto GRAVITY_ACCEL = 9.80665e3f;
constexpr int NUM_STEP_MAX = 1000;
constexpr float PI = 3.14159265359f;
constexpr float RADIUS = 50.0f;
constexpr auto RHO = 1.168e-9; //[kg/mm3]
constexpr float THRES_CONVERGE_POS = 50;
constexpr float THRES_CONVERGE_TIME = 5;

void addRandomUnitVectors(std::vector<Eigen::Vector3f>& vectors, size_t num);

void addUnitVectorsFib(std::vector<Eigen::Vector3f>& vectors, int num);

float MuxMaximumThrust(
	const Eigen::Vector3f& posStart,
	const Eigen::Vector3f& posEnd,
	float duty_max,
	int num_points,
	autd::GeometryPtr geo,
	std::shared_ptr<arfModelLinearBase> arf_model
);

std::shared_ptr<Trajectory> CreateBangbangTrajecotryWithDrag(
	std::shared_ptr<autd::Controller> pAupa,
	const Eigen::Vector3f& posStart,
	const Eigen::Vector3f& posEnd,
	float radius,
	float time_trans_start,
	float duty_max = 1.0f,
	int num_search_points = 10,
	std::shared_ptr<arfModelLinearBase> arf_model = std::make_shared<arfModelFocusSphereExp50mm>()
);


class System;

class System {
private:
	MultiplexManipulator manipulator_;
	FloatingObjectPtr pObject_;
	Eigen::MatrixXf centersAupa_;
	std::vector<Eigen::Matrix3f> rotsAupa_;
	//std::ofstream ofsObs_;
	int step_count_ = 0;
	bool converged_ = false;
	int converge_count_ = 0;
	int steps_resolution_ = 0;
public:
	std::vector<float> times_;
	std::vector<std::pair<size_t, float>> duties_;
public:
	System(
		std::shared_ptr<autd::Controller> pAupa,
		std::shared_ptr<dynaman::Tracker> pTracker,
		FloatingObjectPtr pObject
	);

	void addDriveSequence(
		const float t,
		const Eigen::VectorXf& duty,
		std::vector<float>& times,
		std::vector<std::pair<size_t, float>>& duties
	);

	void check_convergence(const state_type& x, const float t);

	void observe(const state_type& x, const float t);

	void operator()(const state_type& x, state_type& dxdt, const float t);

	bool isConverged();

	void setResolution(int num_steps);
};

bool simulate(
	std::shared_ptr<autd::Controller> pAupa,
	std::shared_ptr<dynaman::Tracker> pTracker,
	std::shared_ptr<dynaman::Trajectory> pTrajectory,
	const Eigen::Vector3f& posError,
	const Eigen::Vector3f& velError,
	float timeInit,
	float timeEnd,
	int stepsPowerResolution = 0,
	std::function<void(const state_type&, const float)> observer = [](const state_type&, const float) {}
);
#endif // !_DYNAMAN_SIMULATOR_HPP