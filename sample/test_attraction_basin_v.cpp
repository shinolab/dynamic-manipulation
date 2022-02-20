#include <ctime>
#include <fstream>
#include <random>
#include <string>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "GainPlan.hpp"
#include "manipulator.hpp"
#include "ThrustSearch.hpp"
#include "haptic_icon.hpp"

using namespace boost::numeric::odeint;
using namespace dynaman;
using state_type = std::vector<float>;

constexpr auto DUTY_MIN = 1.0f / 255.0f;
constexpr float DT_STEP = 0.0005f;
constexpr float DT_OBS = 0.01f;
constexpr float DUTY_FF_MAX = 0.5f;
constexpr auto GRAVITY_ACCEL = 9.80665e3f;
constexpr int NUM_BINARY_SEARCH_MAX = 8;
constexpr int NUM_STEP_MAX = 1000;
constexpr int NUM_WAYPOINTS = 10;
constexpr float NUM_ERROR_COND = 10000;
constexpr float POS_ERROR_MAX = 210;
constexpr float RADIUS = 50.0f;
constexpr auto RHO = 1.168e-9; //[kg/mm3]
constexpr float THRES_CONVERGE_POS = 50;
constexpr float THRES_CONVERGE_TIME = 5;


void addRandomUnitVectors(std::vector<Eigen::Vector3f>& vectors, size_t num) {
	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_real_distribution<> dist(-1, 1);
	for (int i = 0; i < num; i++) {
		float t = std::asinf(dist(mt));
		float u = dist(mt) * 2 * pi - pi;
		vectors.emplace_back(
			cosf(t) * cosf(u),
			cosf(t) * sinf(u),
			sinf(t)
		);
	}
}

float MuxMaximumThrust(
	const Eigen::Vector3f& posStart,
	const Eigen::Vector3f& posEnd,
	float duty_max,
	int num_points,
	autd::GeometryPtr geo,
	std::shared_ptr<arfModelLinearBase> arf_model
) {
	std::vector<MuxThrustSearcher> searchers{
		{geo, arf_model, 1, duty_max},
		{geo, arf_model, 2, duty_max},
		{geo, arf_model, 3, duty_max},
		{geo, arf_model, 4, duty_max}
	};
	float dist = (posEnd - posStart).norm();
	auto direction = (posEnd - posStart).normalized();
	float interval = dist / (num_points - 1);
	float force_min = FLT_MAX;
	for (int i_point = 0; i_point < num_points; i_point++) {
		Eigen::Vector3f pos = posStart + i_point * interval * direction;
		Eigen::MatrixXf posRel = pos.replicate(1, geo->numDevices()) - CentersAutd(geo);
		Eigen::MatrixXf arfMat = arf_model->arf(posRel, RotsAutd(geo));
		float force_max_at_point = 0;
		std::for_each(searchers.begin(), searchers.end(), [&](MuxThrustSearcher& searcher)
			{
				auto duty = searcher.Search(pos, direction);
				auto force = (arfMat * duty).dot(direction);
				if (force > force_max_at_point) {
					force_max_at_point = force;
				}
			}
		);
		if (force_max_at_point < force_min) {
			force_min = force_max_at_point;
		}
	}
	return force_min;
}

std::shared_ptr<Trajectory> CreateBangbangTrajecotryWithDrag(
	std::shared_ptr<autd::Controller> pAupa,
	const Eigen::Vector3f& posStart,
	const Eigen::Vector3f& posEnd,
	float radius,
	float time_trans_start,
	float duty_max = 1.0f,
	int num_search_points = 10,
	std::shared_ptr<arfModelLinearBase> arf_model = std::make_shared<arfModelFocusSphereExp50mm>()
) {
	
	Eigen::Vector3f direction = (posEnd - posStart).normalized();
	auto dist = (posEnd - posStart).norm();
	float mu = 3 * 0.4f / 8.0f / radius;
	float dist_sw = 0.5f / mu * logf(0.5f * (expf(2.f * mu * dist) + 1.0f));

	Eigen::Vector3f pos_sw = posStart + dist_sw * direction;
	float force_accel = MuxMaximumThrust(posStart, pos_sw, duty_max, num_search_points, pAupa->geometry(), arf_model);
	float force_decel = MuxMaximumThrust(posEnd, pos_sw, duty_max, num_search_points, pAupa->geometry(), arf_model);

	std::cout << "force_accel: " << force_accel << std::endl;
	std::cout << "force_decel: " << force_decel << std::endl;
	return TrajectoryBangbangWithDrag::Create(
		std::min(force_accel, force_decel),
		radius,
		static_cast<DWORD>(1000 * time_trans_start),
		posStart,
		posEnd
	);
}

class Logger {
private:
	std::ofstream ofs;
	std::shared_ptr<Trajectory> pTrajectory;
public:
	Logger(const std::string& filename, std::shared_ptr<Trajectory> pTrajectory)
	:pTrajectory(pTrajectory)
	{
		ofs.open(filename);
		ofs << "t,x,y,z,vx,vy,vz,xTgt,yTgt,zTgt" << std::endl;
	}
	~Logger() {
		ofs.close();
	}

	void operator()(const state_type& x, double t) {
		ofs << t;
		for (auto&& e : x) {
			ofs << "," << e;
		}
		auto pos = pTrajectory->pos(static_cast<DWORD>(1000*t));
		ofs << "," << pos.x() << "," << pos.y() << "," << pos.z();
		ofs << std::endl;
	}
};

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
public:
	std::vector<float> times_;
	std::vector<std::pair<size_t, float>> duties_;
public:
	System(
		std::shared_ptr<autd::Controller> pAupa,
		std::shared_ptr<dynaman::Tracker> pTracker,
		FloatingObjectPtr pObject
	):
		manipulator_(pAupa, pTracker),
		pObject_(pObject),
		centersAupa_(CentersAutd(pAupa->geometry())),
		rotsAupa_(RotsAutd(pAupa->geometry()))
	{
		//ofsObs_.open(filename);
	}

	void addDriveSequence(
		const float t,
		const Eigen::VectorXf& duty,
		std::vector<float>& times,
		std::vector<std::pair<size_t, float>> &duties
	) {
		int num_active = (duty.array() > DUTY_MIN).count();
		if (num_active == 0) {
			times.push_back(t);
			duties.push_back(std::make_pair(0, 0));
		}
		else {
			int deviceCount = 0;
			for (int iDevice = 0; iDevice < 11; iDevice++) {
				if (duty[iDevice] > DUTY_MIN) {
					times.push_back(t + DT_OBS / num_active * deviceCount);
					duties.push_back(std::make_pair(iDevice, duty[iDevice]));
					deviceCount++;
				}
			}
		}
	}

	void check_convergence(const state_type& x, const float t) {
		Eigen::Vector3f pos(x[0], x[1], x[2]);
		auto posTgt = pObject_->getPositionTarget(static_cast<DWORD>(1000 * t));
		(pos - posTgt).norm() < THRES_CONVERGE_POS ? converge_count_++ : converge_count_ = 0; 
		converge_count_* DT_STEP > THRES_CONVERGE_TIME ? converged_ = true : converged_ = false;
		//std::cout << "convergence_count :" << converge_count_ << std::endl;
	}

	void observe(const state_type& x, const float t) {
		if (step_count_ % static_cast<int>(std::roundf(DT_OBS / DT_STEP)) == 0) {
			//observation
			DWORD systime_ms = static_cast<DWORD>(1000 * t);
			Eigen::Vector3f pos(x[0], x[1], x[2]);
			pObject_->updateStates(systime_ms, pos);

			//actuation
			Eigen::Vector3f vel, integ;

			pObject_->getStates(pos, vel, integ);
			auto posTgt = pObject_->getPositionTarget(systime_ms);
			auto velTgt = pObject_->getVelocityTarget(systime_ms);
			auto accelTgt = pObject_->getAccelTarget(systime_ms);
			Eigen::Vector3f accelTarget
				= manipulator_.gainP().asDiagonal() * (posTgt - pos)
				+ manipulator_.gainD().asDiagonal() * (velTgt - vel)
				- manipulator_.gainI().asDiagonal() * integ
				+ accelTgt;

			Eigen::Vector3f forceTarget
				= pObject_->totalMass() * accelTarget
				+ pObject_->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, GRAVITY_ACCEL);
			Eigen::MatrixXf posRel = pos.replicate(1, 11) - centersAupa_;

			Eigen::VectorXf duty = manipulator_.ComputeDuty(forceTarget, pos);
			int num_active = (duty.array() > DUTY_MIN).count();
			duty = (duty * num_active).cwiseMin(1.0f).cwiseMax(0.0f);
			addDriveSequence(
				t,
				duty,
				times_,
				duties_
			);
		}
		step_count_++;
	}

	void operator()(const state_type& x, state_type& dxdt, const float t) {
		//observation
		DWORD systime_ms = static_cast<DWORD>(t * 1000);
		Eigen::Vector3f pos(x[0], x[1], x[2]);
		Eigen::Vector3f vel(x[3], x[4], x[5]);

		Eigen::MatrixXf posRel = pos.replicate(1, 11) - centersAupa_;

		size_t iDevice = 0;
		float duty = 0;
		for (auto itr = times_.rbegin(); itr != times_.rend(); itr++) {
			if (*itr <= t) {
				auto it = std::distance(itr, times_.rend())-1;
				iDevice = duties_[it].first;
				duty = duties_[it].second;
				break;
			}
		}

		Eigen::MatrixXf FTrue = manipulator_.arfModel()->arf(posRel, rotsAupa_);
		Eigen::Vector3f forceArf = FTrue.col(iDevice)* duty;
		Eigen::Vector3f drag = -0.5f * pi * RHO * vel.norm() * vel.norm() * pObject_->Radius() * pObject_->Radius() * vel.normalized();
		Eigen::Vector3f forceTotal = forceArf + drag;
		Eigen::Vector3f accelResult = forceTotal / pObject_->totalMass();
		dxdt[0] = x[3];
		dxdt[1] = x[4];
		dxdt[2] = x[5];
		dxdt[3] = accelResult[0];
		dxdt[4] = accelResult[1];
		dxdt[5] = accelResult[2];
	}

	bool isConverged() {
		return converged_;
	}
};

bool simulate(
	std::shared_ptr<autd::Controller> pAupa,
	std::shared_ptr<dynaman::Tracker> pTracker,
	std::shared_ptr<dynaman::Trajectory> pTrajectory,
	const Eigen::Vector3f& posError,
	const Eigen::Vector3f& velError,
	float timeInit,
	float timeEnd
) {
	auto systimeInit = static_cast<DWORD>(timeInit);

	FloatingObjectPtr pObject = FloatingObject::Create(
		pTrajectory->pos(systimeInit),
		Eigen::Vector3f::Constant(-500),
		Eigen::Vector3f::Constant(500),
		RADIUS
	);

	System system(pAupa, pTracker, pObject);

	pObject->SetTrajectory(pTrajectory);

	state_type state(6);
	state[0] = pTrajectory->pos(systimeInit).x() + posError.x();
	state[1] = pTrajectory->pos(systimeInit).y() + posError.y();
	state[2] = pTrajectory->pos(systimeInit).z() + posError.z();
	state[3] = pTrajectory->vel(systimeInit).x() + velError.x();
	state[4] = pTrajectory->vel(systimeInit).y() + velError.y();
	state[5] = pTrajectory->vel(systimeInit).z() + velError.z();

	runge_kutta4<state_type> stepper;
	auto steps = integrate_const(
		stepper,
		std::ref(system),
		state,
		timeInit,
		timeEnd,
		DT_STEP,
		[&system](const state_type& x, float t) {
			system.observe(x, t);
			system.check_convergence(x, t);
		}
	);

	//std::cout << "total steps: " << steps << std::endl;
	//std::cout << "terminal state: ";
	//for (auto&& e : state) {
	//	std::cout << e << "," ;
	//}
	//std::cout << std::endl;
	return system.isConverged();
}


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
	Eigen::Vector3f posInitTgt = 200 * Eigen::Vector3f(1, 0, -1).normalized();
	Eigen::Vector3f posEndTgt = 200 * Eigen::Vector3f(1, 0, 1).normalized();

	std::cout << "posInitTgt: " << posInitTgt.transpose() << std::endl;
	std::cout << "posEndTgt: " << posEndTgt.transpose() << std::endl;

	// ***********************************
	char date[sizeof("yymmdd_HHMMSS")];
	auto tt = std::time(nullptr);
	std::strftime(&date[0], sizeof(date), "%y%m%d_%H%M%S", std::localtime(&tt));
	std::string prefix(date);
	std::string filename = prefix + "_v" + std::string(argv[1]) + "_basin_summery_v.csv";
	std::string filenameLog = prefix + "_v" + std::string(argv[1]) + "_log_v.txt";
	std::ofstream ofs(filename);
	std::ofstream ofsLog(filenameLog);
	
	ofs << "dv";
	for (int it = 0; it < NUM_WAYPOINTS; it++) {
		ofs << ",t=" << it;
	}
	ofs << std::endl;

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

	auto start = std::chrono::system_clock::now();
	for (auto&& veLv : velErrorLevels){
		ofs << veLv;

		for (int it = 0; it < NUM_WAYPOINTS; it++) {
			float posErrorLb = 0;
			float posErrorUb = POS_ERROR_MAX;
			float posErrorMid;

			for (int ib = 0; ib < NUM_BINARY_SEARCH_MAX; ib++) {
				auto convergedInAllTrials = true;
				posErrorMid = 0.5 * (posErrorLb + posErrorUb);
				if (posErrorUb - posErrorMid < 1 || posErrorMid - posErrorLb < 1) {
					break;
				}
				std::vector<Eigen::Vector3f> velErrors;
				std::vector<Eigen::Vector3f> posErrors;
				addRandomUnitVectors(velErrors, NUM_ERROR_COND);
				addRandomUnitVectors(posErrors, NUM_ERROR_COND);

				for (int ic = 0; ic < NUM_ERROR_COND; ic++) {
					auto pe = posErrorMid * posErrors[ic];
					auto ve = veLv * velErrors[ic];
					const float timeInit = timeTransStart + it * timeToTrans / NUM_WAYPOINTS;
					const float timeEnd = timeInit + 10.0f;
					auto isConverged = simulate(pAupa, pTracker, pTrajectory, pe, ve, timeInit, timeEnd);
					//std::cout << "isConverged: " << std::boolalpha << isConverged << std::endl;
					if (!isConverged) {
						convergedInAllTrials = false;
						std::cout
							<< "failed to converge: (posError: " << pe.transpose()
							<< ", velError: " << ve.transpose() << ")"
							<< "(norm:" << pe.norm() << ", " << ve.norm() << ")" << std::endl;
						ofsLog
							<< "failed to converge: (posError: " << pe.transpose()
							<< ", velError: " << ve.transpose() << ")"
							<< "(norm:" << pe.norm() << ", " << ve.norm() << ")" << std::endl;

						break;
					}
				}
				if (convergedInAllTrials) {
					ofsLog << "converged in all trials: posErrorLv: " << posErrorMid << "mm @ v=" << veLv << "mm/s, waypoint: " << it << ")" << std::endl;
					std::cout <<"converged in all trials: posErrorLv: " << posErrorMid << "mm @ v=" << veLv << "mm/s, waypoint: " << it << ")" << std::endl;
				}
				convergedInAllTrials ? posErrorLb = posErrorMid : posErrorUb = posErrorMid;
			}
			ofsLog << "Tolerant pos error is:" << posErrorMid << " mm @ v=" << veLv << "mm/s, waypoint: " << it << std::endl;
			std::cout << "Tolerant pos error is:" << posErrorMid <<" mm @ v=" << veLv << "mm/s, waypoint: " << it  << std::endl;
			ofs << "," << posErrorMid;
		}
		ofs << std::endl;
		auto end = std::chrono::system_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		ofsLog << "elapsed:" << elapsed.count() << "ms" << std::endl;
		std::cout << "elapsed:" << elapsed.count() << "ms" << std::endl;
	}
	ofs.close();
	ofsLog.close();

	return 0;
}