#include <ctime>
#include <fstream>
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

constexpr auto GRAVITY_ACCEL = 9.80665e3f; 
constexpr auto RHO = 1.168e-9; //[kg/mm3]
constexpr auto DUTY_MIN = 1.0f / 255.0f;
constexpr float DT_MUX = 0.0025f;
constexpr int NUM_STEP_MAX = 1000;
constexpr float DT_STEP = 0.0005f;
constexpr float DT_OBS = 0.01f;

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
	std::ofstream ofsObs_;
	int step_count_ = 0;
public:
	std::vector<float> times_;
	std::vector<std::pair<size_t, float>> duties_;
public:
	System(
		std::shared_ptr<autd::Controller> pAupa,
		std::shared_ptr<dynaman::Tracker> pTracker,
		FloatingObjectPtr pObject,
		const std::string& filename
	):
		manipulator_(pAupa, pTracker),
		pObject_(pObject),
		centersAupa_(CentersAutd(pAupa->geometry())),
		rotsAupa_(RotsAutd(pAupa->geometry()))
	{
		ofsObs_.open(filename);
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
			ofsObs_
				<< t
				<< "," << pos.x() << "," << pos.y() << "," << pos.z()
				<< "," << vel.x() << "," << vel.y() << "," << vel.z()
				<< "," << integ.x() << "," << integ.y() << "," << integ.z();
			for (int i = 0; i < duty.size(); i++) {
				ofsObs_ << "," << duty[i];
			}
			ofsObs_ << std::endl;
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
};

int main(int argc, char** argv) {

	//Experiment Condition**********************

	Eigen::Vector3f posInitTgt(-200, 0, 0);
	Eigen::Vector3f posEndTgt(200, 0, 0);
	Eigen::Vector3f posError(0, 0, 100);
	Eigen::Vector3f velError(0, 0, 0);
	state_type state(6);
	state[0] = posInitTgt.x() + posError.x();
	state[1] = posInitTgt.y() + posError.y();
	state[2] = posInitTgt.z() + posError.z();
	state[3] = velError[0];
	state[4] = velError[1];
	state[5] = velError[2];

	float time_init = 1.0f;
	float time_trans_start = 2.0f;
	float time_end = 4.0f;
	float duty_ff_max = 0.5f;


	// ***********************************
	char date[sizeof("yymmdd_HHMMSS")];
	auto tt = std::time(nullptr);
	std::strftime(&date[0], sizeof(date), "%y%m%d_%H%M%S", std::localtime(&tt));
	std::string prefix(date);
	std::string filenameTrue = prefix + "_basin_true_log.csv";
	std::string filenameObs = prefix + "_basin_obs_log.csv";

	auto pAupa = haptic_icon::CreateController();
	auto pTracker = haptic_icon::CreateTracker("blue_target_r50mm.png");
	FloatingObjectPtr pObject = FloatingObject::Create(
		Eigen::Vector3f::Zero(),
		Eigen::Vector3f::Constant(-500),
		Eigen::Vector3f::Constant(500),
		50.0f
	);
	System system(pAupa, pTracker, pObject, filenameObs);
	
	auto pTrajectory = CreateBangbangTrajecotryWithDrag(
		pAupa,
		posInitTgt,
		posEndTgt,
		pObject->Radius(),
		time_trans_start,
		duty_ff_max
	);
	
	pObject->SetTrajectory(pTrajectory);

	Logger logger(filenameTrue, pTrajectory);

	runge_kutta4<state_type> stepper;
	auto steps = integrate_const(
		stepper,
		std::ref(system),
		state,
		time_init,
		time_end,
		DT_STEP,
		[&system, &logger](const state_type& x, const float t) {
			logger(x, t);
			system.observe(x, t);
		}
	);
	
	std::cout << "Propagation finished. Total number of steps: " << steps << std::endl;
	std::cout << "Terminal state: ";
	for (auto&& e : state) {
		std::cout << e << ",";
	}
	std::cout << std::endl;

	return 0;
}