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
		ofs << "t,x,y,z,vx,vy,vz,ix,iy,iz,xTgt,yTgt,zTgt" << std::endl;
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
	}

	void operator()(const state_type& x, state_type& dxdt, const float t) {
		//previous state is saved in FloatingObject
		//observation
		DWORD systime_ms = static_cast<DWORD>(t * 1000);
		Eigen::Vector3f pos_true(x[0], x[1], x[2]);
		Eigen::Vector3f vel_true(x[3], x[4], x[5]);
		Eigen::Vector3f integ_true(x[6], x[7], x[8]);

		//actuation
		auto posTgt = pObject_->getPositionTarget(systime_ms);
		auto velTgt = pObject_->getVelocityTarget(systime_ms);
		auto accelTgt = pObject_->getAccelTarget(systime_ms);
		Eigen::Vector3f accelTarget
			= manipulator_.gainP().asDiagonal() * (posTgt - pos_true)
			+ manipulator_.gainD().asDiagonal() * (velTgt - vel_true)
			- manipulator_.gainI().asDiagonal() * integ_true
			+ accelTgt;

		Eigen::Vector3f forceTarget
			= pObject_->totalMass() * accelTarget
			+ pObject_->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, GRAVITY_ACCEL);
		Eigen::MatrixXf posRelTrue = pos_true.replicate(1, 11) - centersAupa_;

		Eigen::VectorXf duty = manipulator_.ComputeDuty(forceTarget, pos_true);
		int num_active = (duty.array() > DUTY_MIN).count();
		duty = (duty * num_active).cwiseMin(1.0f).cwiseMax(0.0f);
		Eigen::MatrixXf FTrue = manipulator_.arfModel()->arf(posRelTrue, rotsAupa_);

		Eigen::Vector3f forceArf = FTrue * duty;
		Eigen::Vector3f drag = -0.5f * pi * RHO * vel_true.norm() * vel_true.norm() * pObject_->Radius() * pObject_->Radius() * vel_true.normalized();
		Eigen::Vector3f forceTotal = forceArf + drag;
		Eigen::Vector3f accelResult = forceTotal / pObject_->totalMass();
		dxdt[0] = x[3];
		dxdt[1] = x[4];
		dxdt[2] = x[5];
		dxdt[3] = accelResult[0];
		dxdt[4] = accelResult[1];
		dxdt[5] = accelResult[2];
		dxdt[6] = x[0] - posTgt.x();
		dxdt[7] = x[1] - posTgt.y();
		dxdt[8] = x[2] - posTgt.z();
	}
};

int main(int argc, char** argv) {

	//Experiment Condition**********************

	Eigen::Vector3f posInitTgt(-200, 0, 0);
	Eigen::Vector3f posEndTgt(200, 0, 0);
	Eigen::Vector3f posError(0, 0, 0);
	Eigen::Vector3f velError(0, 0, 0);
	state_type state(9);
	state[0] = posInitTgt.x() + posError.x();
	state[1] = posInitTgt.y() + posError.y();
	state[2] = posInitTgt.z() + posError.z();
	state[3] = velError[0];
	state[4] = velError[1];
	state[5] = velError[2];
	state[6] = 0;
	state[7] = 0;
	state[8] = 0;

	float time_init = 1.0f;
	float time_trans_start = 2.0f;
	float time_end = 5.0f;
	float duty_ff_max = 0.5f;

	constexpr int NUM_STEP_MAX = 1000;
	constexpr float DT_STEP = 0.001f;

	// ***********************************

	auto pAupa = haptic_icon::CreateController();
	auto pTracker = haptic_icon::CreateTracker("blue_target_r50mm.png");
	FloatingObjectPtr pObject = FloatingObject::Create(
		Eigen::Vector3f::Zero(),
		Eigen::Vector3f::Constant(-500),
		Eigen::Vector3f::Constant(500),
		50.0f
	);
	System system(pAupa, pTracker, pObject);
	
	auto pTrajectory = CreateBangbangTrajecotryWithDrag(
		pAupa,
		posInitTgt,
		posEndTgt,
		pObject->Radius(),
		time_trans_start,
		duty_ff_max
	);
	
	pObject->SetTrajectory(pTrajectory);

	char date[sizeof("yymmdd_HHMMSS")];
	auto tt = std::time(nullptr);
	std::strftime(&date[0], sizeof(date), "%y%m%d_%H%M%S", std::localtime(&tt));
	std::string filename(date);
	filename += "basin_log.csv";
	Logger logger(filename, pTrajectory);


	runge_kutta4<state_type> stepper;
	auto steps = integrate_const(
		stepper,
		std::ref(system),
		state,
		time_init,
		time_end,
		DT_STEP,
		std::ref(logger)
	);
	
	std::cout << "Propagation finished. Total number of steps: " << steps << std::endl;
	std::cout << "Terminal state: ";
	for (auto&& e : state) {
		std::cout << e << ",";
	}
	std::cout << std::endl;

	return 0;
}