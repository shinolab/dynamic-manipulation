#include <random>
#include <boost/numeric/odeint.hpp>
#include "dynaman_simulator.hpp"
#include "ThrustSearch.hpp"

using namespace boost::numeric::odeint;

void addRandomUnitVectors(std::vector<Eigen::Vector3f>& vectors, size_t num) {
	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_real_distribution<> dist(-1, 1);
	for (int i = 0; i < num; i++) {
		float t = std::asinf(dist(mt));
		float u = dist(mt) * 2 * PI - PI;
		vectors.emplace_back(
			cosf(t) * cosf(u),
			cosf(t) * sinf(u),
			sinf(t)
		);
	}
}

void addUnitVectorsFib(std::vector<Eigen::Vector3f>& vectors, int num) {
	const float psi = (std::sqrtf(5.0f) - 1.0f) / 2.0f;
	for (int i = -num; i <= num; i++) {
		auto theta = std::asinf(2.0f * i / (2.0f * num + 1.0f));
		auto phi = 2.0f * PI * i * psi;
		//std::cout << "theta: " << theta << ", phi: " << phi << std::endl;
		vectors.emplace_back(
			std::cosf(theta) * std::cosf(phi),
			std::cosf(theta) * std::sinf(phi),
			std::sinf(theta)
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
	float duty_max,
	int num_search_points,
	std::shared_ptr<arfModelLinearBase> arf_model
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

System::System(
	std::shared_ptr<autd::Controller> pAupa,
	std::shared_ptr<dynaman::Tracker> pTracker,
	FloatingObjectPtr pObject
) :
	manipulator_(pAupa, pTracker),
	pObject_(pObject),
	centersAupa_(CentersAutd(pAupa->geometry())),
	rotsAupa_(RotsAutd(pAupa->geometry()))
{
	//ofsObs_.open(filename);
}

void System::addDriveSequence(
	const float t,
	const Eigen::VectorXf& duty,
	std::vector<float>& times,
	std::vector<std::pair<size_t, float>>& duties
) {
	int num_active = (duty.array() > DUTY_MIN).count();
	if (num_active == 0) {
		times.push_back(t);
		duties.push_back(std::make_pair(0, 0));
	}
	return;
	
	int deviceCount = 0;
	for (int iDevice = 0; iDevice < 11; iDevice++) {
		if (duty[iDevice] > DUTY_MIN) {
			times.push_back(t + DT_OBS / num_active * deviceCount);
			duties.push_back(std::make_pair(iDevice, duty[iDevice]));
			deviceCount++;
		}
	}
	return;
}

void System::checkConvergence(const state_type& x, const float t) {
	Eigen::Vector3f pos(x[0], x[1], x[2]);
	auto posTgt = pObject_->getPositionTarget(static_cast<DWORD>(1000 * t));
	(pos - posTgt).norm() < THRES_CONVERGE_POS ? converge_count_++ : converge_count_ = 0;
	converge_count_* DT_STEP > THRES_CONVERGE_TIME ? converged_ = true : converged_ = false;
}

void System::observe(const state_type& x, const float t) {
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
		if (steps_resolution_ > 0) {
			float resolution = 1.0f / steps_resolution_;
			duty = ((duty / resolution).array().round() * resolution).matrix();
		}
		addDriveSequence(
			t,
			duty,
			times_,
			duties_
		);
	}
	step_count_++;
}

Eigen::VectorXf System::aupaPower(float time_s) {
	size_t iDevice = 0;
	float duty = 0;
	int num_device = centersAupa_.cols();
	Eigen::VectorXf power = Eigen::VectorXf::Zero(num_device);
	for (auto itr = times_.rbegin(); itr != times_.rend(); itr++) {
		if (*itr <= time_s) {
			auto it = std::distance(itr, times_.rend()) - 1;
			iDevice = duties_[it].first;
			power[iDevice] = duties_[it].second;
			break;
		}
	}
	return power;
}

void System::operator()(const state_type& x, state_type& dxdt, const float t) {
	//observation
	Eigen::Vector3f pos(x[0], x[1], x[2]);
	Eigen::Vector3f vel(x[3], x[4], x[5]);
	Eigen::MatrixXf posRel = pos.replicate(1, 11) - centersAupa_;
	Eigen::Vector3f forceArf = manipulator_.arfModel()->arf(posRel, rotsAupa_) * aupaPower(t);
	Eigen::Vector3f drag = -0.5f * PI * RHO * vel.norm() * vel.norm() * pObject_->Radius() * pObject_->Radius() * vel.normalized();
	Eigen::Vector3f accel = (forceArf + drag) / pObject_->totalMass();
	dxdt[0] = x[3];
	dxdt[1] = x[4];
	dxdt[2] = x[5];
	dxdt[3] = accel[0];
	dxdt[4] = accel[1];
	dxdt[5] = accel[2];
}

bool System::isConverged() {
	return converged_;
}

void System::setResolution(int num_steps) {
	steps_resolution_ = num_steps;
}

Simulator::Simulator(
	std::shared_ptr<autd::Controller> pAupa,
	std::shared_ptr<dynaman::Tracker> pTracker,
	FloatingObjectPtr pObject,
	int num_resolution
):
	m_system(pAupa, pTracker, pObject),
	m_pObject(pObject)
{
	m_system.setResolution(num_resolution);
}

int Simulator::integrate(
	state_type& state,
	float time_init,
	float time_end,
	std::function<void(const state_type& x, const float t)>& observer
) {
	boost::numeric::odeint::runge_kutta4<state_type> stepper;

	auto steps = integrate_const(
		stepper,
		std::ref(system),
		state,
		time_init,
		time_init,
		DT_STEP,
		[this, &observer](const state_type& x, float t) {
			m_system.observe(x, t);
			checkConvergence(x, t);
			observer(x, t);
		}
	);
	return steps;
}

void Simulator::checkConvergence(const state_type& x, const float time_s) {
	Eigen::Vector3f pos(x[0], x[1], x[2]);
	auto posTgt = m_pObject->getPositionTarget(static_cast<DWORD>(1000 * time_s));
	(pos - posTgt).norm() < THRES_CONVERGE_POS ? m_converge_count++ : m_converge_count = 0;
	m_converge_count * DT_STEP > THRES_CONVERGE_TIME ? m_converged = true : m_converged = false;
}

bool Simulator::isConverged() {
	return m_converged;
}