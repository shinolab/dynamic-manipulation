#include <ctime>
#include <fstream>
#include <string>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "GainPlan.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"

using namespace boost::numeric::odeint;
using namespace dynaman;
using state_type = std::vector<float>;

constexpr auto GRAVITY_ACCEL = 9.80665e3f; 
constexpr auto RHO = 1.168e-9; //[kg/mm3]


class Logger {
public:
	std::ofstream ofs;

	Logger(const std::string& filename) {
		ofs.open(filename);
		ofs << "t,x,y,z,vx,vy,vz,ix,iy,iz" << std::endl;
	}
	~Logger() {
		ofs.close();
	}

	void operator()(const state_type& x, double t) {
		ofs << t;
		for (auto&& e : x) {
			ofs << "," << e;
		}
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

	void F(const Eigen::Vector3f& pos) {
		;
		Eigen::MatrixXf posRel = pos.replicate(1, 11) - centersAupa_;
		std::cout << "posRel:" << std::endl << posRel << std::endl;
		auto F = manipulator_.arfModel()->arf(posRel, rotsAupa_);
		std::cout << "F:" << std::endl << F << std::endl;
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
		Eigen::MatrixXf FTrue = manipulator_.arfModel()->arf(posRelTrue, rotsAupa_);

		Eigen::Vector3f forceArf = FTrue * duty;
		Eigen::Vector3f drag = -0.5f * pi * RHO * vel_true.norm() * vel_true.norm() * pObject_->Radius() * pObject_->Radius() * vel_true.normalized();
		Eigen::Vector3f forceTotal = forceArf + drag;
		Eigen::Vector3f accelResult = forceTotal / pObject_->totalMass();
		if (t > 4.22) {
			std::cout
				<< "pos_true: " << pos_true.transpose()
				<< std::endl << "vel_true: " << vel_true.transpose()
				<< std::endl << "integ: " << integ_true.transpose()
				<< std::endl << "forceTarget: " << forceTarget.transpose()
				<< std::endl << "duty: " << duty.transpose() << std::endl
				<< std::endl << "F: " << std::endl << FTrue
				<< std::endl;
			std::cout << "forceArf: " << forceArf.transpose() << std::endl;
			std::cout << "drag: " << drag.transpose() << std::endl;

		}
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

	Eigen::Vector3f posInitTgt(0, 0, 0);
	Eigen::Vector3f posEndTgt(0, 0, 0);
	Eigen::Vector3f posError(100, 0, 0);
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
	float time_end = 4.27f;

	constexpr int NUM_STEP_MAX = 1000;
	constexpr float DT_STEP = 0.001f;

	// ***********************************


	char date[sizeof("yymmdd_HHMMSS")];
	auto tt = std::time(nullptr);
	std::strftime(&date[0], sizeof(date), "%y%m%d_%H%M%S", std::localtime(&tt));
	std::string filename(date);
	filename += "basin_log.csv";
	Logger logger(filename);

	auto pAupa = haptic_icon::CreateController();
	auto pTracker = haptic_icon::CreateTracker("blue_target_r50mm.png");
	FloatingObjectPtr pObject = FloatingObject::Create(
		Eigen::Vector3f::Zero(),
		Eigen::Vector3f::Constant(-500),
		Eigen::Vector3f::Constant(500),
		50.0f
	);
	System system(pAupa, pTracker, pObject);
	Eigen::Vector3f p(0.386947, 5.94529e-07, -9.65365e-09);
	Eigen::Vector3f p2(0.386947, 0.1, 0);
	system.F(p2);
	return 0;
	//pObject->SetTrajectory(
	//	TrajectoryBangbangWithDrag::Create(
	//		1.0,
	//		pObject->Radius(),
	//		static_cast<DWORD>(1000 * time_trans_start),
	//		posInitTgt,
	//		posEndTgt
	//	)
	//);

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