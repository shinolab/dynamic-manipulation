#include <algorithm>
#include <future>
#include <iomanip>
#include "MathUtil.hpp"
#include "QPSolver.h"
#include "manipulator.hpp"
#include "geometryUtil.hpp"
#include "ThrustSearch.hpp"

namespace dynaman {

	constexpr DWORD THRES_TIMEOUT_TRACK_MS = 100;
	constexpr float DUTY_MIN = 1.0f/255.f;
	constexpr auto GRAVITY_ACCEL = 9.80665e3f;//[mm/s2]
	constexpr float THRES_MIN_INNERPRODUCT = 0.17f;

	MultiplexManipulator::MultiplexManipulator(
		std::shared_ptr<autd::Controller> pAupa,
		std::shared_ptr<dynaman::Tracker> pTracker,
		const Eigen::Vector3f& gainP,
		const Eigen::Vector3f& gainD,
		const Eigen::Vector3f& gainI,
		float freqLm,
		int period_control_ms,
		int period_track_ms,
		float lambda,
		std::shared_ptr<arfModelLinearBase> arfModelPtr
	):
		m_pAupa(pAupa),
		m_pTracker(pTracker),
		m_gainP(gainP),
		m_gainD(gainD),
		m_gainI(gainI),
		m_freqLm(freqLm),
		m_lambda(lambda),
		m_period_control_ms(period_control_ms),
		m_period_track_ms(period_track_ms),
		m_arfModelPtr(arfModelPtr),
		m_isRunning(false),
		m_isPaused(false),
		m_logEnabled(false)
	{}

	std::shared_ptr<Manipulator> MultiplexManipulator::Create(
		std::shared_ptr<autd::Controller> pAupa,
		std::shared_ptr<dynaman::Tracker> pTracker,
		const Eigen::Vector3f& gainP,
		const Eigen::Vector3f& gainD,
		const Eigen::Vector3f& gainI,
		float freqLm,
		int period_control_ms,
		int period_track_ms,
		float lambda,
		std::shared_ptr<arfModelLinearBase> arfModelPtr
	) {
		return std::make_shared<MultiplexManipulator>(
			pAupa,
			pTracker,
			gainP,
			gainD,
			gainI,
			freqLm,
			period_control_ms,
			period_track_ms,
			lambda,
			arfModelPtr
			);
	}

	int MultiplexManipulator::StartManipulation(
		FloatingObjectPtr pObject
	) {
		if (!m_pTracker->isOpen()) {
			std::cerr << "ERROR: Tracker is NOT open!" << std::endl;
			return -1;
		}
		if (!m_pAupa->isOpen()) {
			std::cerr << "ERROR: AUPA controller is not open." << std::endl;
			return -1;
		}
		m_pObject = pObject;

		if (m_logEnabled) {
			InitLogStream();
		}

		timeBeginPeriod(1);
		{
			std::lock_guard<std::shared_mutex> lock(m_mtx_isRunning);
			m_isRunning = true;
		}
		m_thr_track = std::thread([this]()
			{
				while (this->IsRunning()) {
					auto time_loop_init_ms = timeGetTime();
					this->ExecuteSingleObservation();
					int wait_time_ms = m_period_track_ms - (timeGetTime() - time_loop_init_ms);
					Sleep(std::clamp(wait_time_ms, 0, m_period_track_ms));
				}
			}
		);
		m_thr_control = std::thread([this]()
			{
				while (this->IsRunning()) {
					DWORD timeLoopInit = timeGetTime();
					this->ExecuteSingleActuation();
					int wait_time_ms = m_period_control_ms - (timeGetTime() - timeLoopInit);
					Sleep(std::clamp(wait_time_ms, 0, m_period_control_ms));
				}
			}
		);
		timeEndPeriod(1);
		return 0;
	}

	void MultiplexManipulator::FinishManipulation() {
		{
			std::lock_guard<std::shared_mutex> lk(m_mtx_isRunning);
			m_isRunning = false;
		}
		if (m_thr_control.joinable()) {
			m_thr_control.join();
		}
		if (m_thr_track.joinable()) {
			m_thr_track.join();
		}
		if (m_obsLogStream.is_open()) {
			m_obsLogStream.close();
		}
		if (m_controlLogStream.is_open()) {
			m_controlLogStream.close();
		}
		m_pAupa->AppendGainSync(autd::NullGain::Create());
	}

	void MultiplexManipulator::PauseManipulation() {
		std::lock_guard<std::mutex> lock(m_mtx_isPaused);
		m_isPaused = true;
	}

	void MultiplexManipulator::ResumeManipulation() {
		std::lock_guard<std::mutex> lock(m_mtx_isPaused);
		m_isPaused = false;
	}

	void MultiplexManipulator::SetOnPause(std::function<void()>& func_on_pause) {
		m_on_pause = func_on_pause;
	}

	Eigen::Vector3f MultiplexManipulator::ComputeForce(DWORD systime, FloatingObjectPtr pObject) {
		Eigen::Vector3f pos, vel, integ, posTgt, velTgt, accelTgt;
		m_pObject->getStates(pos, vel, integ);
		m_pObject->getStatesTarget(posTgt, velTgt, accelTgt, systime);

		Eigen::Vector3f accel
				= gainP().asDiagonal() * (posTgt - pos)
				+ gainD().asDiagonal() * (velTgt - vel)
				- gainI().asDiagonal() * integ
				+ accelTgt;

		Eigen::Vector3f force
			= m_pObject->totalMass() * accel
			+ m_pObject->weight() * Eigen::Vector3f(0.f, 0.f, GRAVITY_ACCEL);
		return force;
	}

	QuadraticProgram MultiplexManipulator::constructQp(
		const Eigen::Vector3f& forceTarget,
		const Eigen::MatrixXf& posRel,
		const std::vector<Eigen::Matrix3f>& rotsAupa
	) {
		const int num_device_to_use = posRel.cols();
		Eigen::MatrixXf F = m_arfModelPtr->arf(posRel, rotsAupa);
		Eigen::VectorXi condEq(1);
		condEq << -1;
		Eigen::MatrixXf A(1, num_device_to_use);
		A << Eigen::RowVectorXf::Ones(num_device_to_use);
		Eigen::VectorXf b(1);
		b << 1;
		return QuadraticProgram(
			A,
			b,
			F.transpose() * F + m_lambda * Eigen::MatrixXf::Identity(num_device_to_use, num_device_to_use),
			-F.transpose() * forceTarget,
			condEq,
			Eigen::VectorXf::Zero(num_device_to_use),
			Eigen::VectorXf::Ones(num_device_to_use)
		);
	}

	Eigen::Array<bool, -1, -1> MultiplexManipulator::chooseAupaToDrive(
		const Eigen::Vector3f& force,
		const Eigen::Vector3f& position
	) {
		Eigen::MatrixXf posRel = position.replicate(1, m_pAupa->geometry()->numDevices()) - CentersAutd(m_pAupa->geometry());
		Eigen::MatrixXf directions = posRel.colwise().normalized();
		Eigen::Array<bool, -1, -1> isActive = ((directions.transpose() * force.normalized()).array() > THRES_MIN_INNERPRODUCT);
		return isActive;
	}

	Eigen::MatrixXf MultiplexManipulator::posRelActive(
		const Eigen::Vector3f& position,
		const Eigen::Array<bool, -1, -1>& isActive
	) {
		Eigen::MatrixXf posRel = position.replicate(1, m_pAupa->geometry()->numDevices()) - CentersAutd(m_pAupa->geometry());
		Eigen::MatrixXf posRelActive(3, isActive.count());
		int iAupaActive = 0;
		for (int iAupa = 0; iAupa < posRel.cols(); iAupa++) {
			if (isActive(iAupa)) {
				posRelActive.col(iAupaActive) << posRel.col(iAupa);
				iAupaActive++;
			}
		}
		return posRelActive;
	}

	std::vector<Eigen::Matrix3f> MultiplexManipulator::rotsAupaActive(
		const Eigen::Array<bool, -1, -1>& isActive
	) {
		std::vector<Eigen::Matrix3f> rotsAutd = RotsAutd(m_pAupa->geometry());
		std::vector<Eigen::Matrix3f> rotsAutdActive(isActive.count());
		int num_device = m_pAupa->geometry()->numDevices();
		int iAupaActive = 0;
		for (int iAupa = 0; iAupa < num_device; iAupa++) {
			if (isActive(iAupa)) {
				rotsAutdActive[iAupaActive] = rotsAutd[iAupa];
				iAupaActive++;
			}
		}
		return rotsAutdActive;
	}

	Eigen::VectorXf MultiplexManipulator::ComputeDuty(
		const Eigen::Vector3f& force,
		const Eigen::Vector3f& position
	) {
		auto isActive = chooseAupaToDrive(force, position);
		if (isActive.count() == 0) {
			return Eigen::VectorXf::Zero(m_pAupa->geometry()->numDevices());
		}
		auto qp = constructQp(
			force,
			posRelActive(position, isActive),
			rotsAupaActive(isActive)
		);
		auto resultActive = qp.solve().solution;
		auto resultFull = expandDuty(resultActive, isActive);
		return resultFull;
	}
	
	Eigen::VectorXf MultiplexManipulator::expandDuty(
		const Eigen::VectorXf& duties_active,
		const Eigen::Array<bool, -1, -1>& is_active
	) {
		int iAupaActive = 0;
		int num_aupa = m_pAupa->geometry()->numDevices();
		Eigen::VectorXf resultFull = Eigen::VectorXf::Zero(num_aupa);
		for (int i_aupa_full = 0; i_aupa_full < num_aupa; i_aupa_full++) {
			if (is_active(i_aupa_full)) {
				resultFull(i_aupa_full) = duties_active(iAupaActive);
				iAupaActive++;
			}
		}
		return resultFull;
	}

	std::vector<autd::GainPtr> MultiplexManipulator::CreateLateralGainList(
		const Eigen::VectorXf& duties,
		const Eigen::Vector3f& focus
	) {
		std::vector<float> dutiesStl(duties.size());
		Eigen::Map<Eigen::VectorXf>(&dutiesStl[0], duties.size()) = duties;
		int num_active = (duties.array() > DUTY_MIN).count();
		std::vector<autd::GainPtr> gain_list(num_active);
		int id_begin_search = 0;
		for (auto itr_list = gain_list.begin(); itr_list != gain_list.end(); itr_list++) {
			std::map<int, autd::GainPtr> gain_map;
			auto itr_duties = std::find_if(dutiesStl.begin() + id_begin_search, dutiesStl.end(), [](float u) {return u > DUTY_MIN; });
			for (int i_autd = 0; i_autd < m_pAupa->geometry()->numDevices(); i_autd++) {
				if (i_autd == std::distance(dutiesStl.begin(), itr_duties)) {
					int amplitude = std::clamp(static_cast<int>((*itr_duties) * 255.f * num_active), 0, 255);
					gain_map.insert(std::make_pair(i_autd, autd::FocalPointGain::Create(focus, amplitude)));
				}
				else {
					gain_map.insert(std::make_pair(i_autd, autd::NullGain::Create()));
				}
			}
			*itr_list = autd::GroupedGain::Create(gain_map);
			id_begin_search = std::distance(dutiesStl.begin(), itr_duties) + 1;
		}
		return gain_list;
	}

	void MultiplexManipulator::ApplyActuation(const Eigen::VectorXf& duties) {
		int num_active = (duties.array() > DUTY_MIN).count();
		if (num_active == 0) {
			m_pAupa->AppendGainSync(autd::NullGain::Create());
		}
		else {
			auto gain_list = CreateLateralGainList(duties, m_pObject->position());
			m_pAupa->ResetLateralGain();
			m_pAupa->AppendLateralGain(gain_list);
			m_pAupa->StartLateralModulation(m_freqLm);
		}
	}

	void MultiplexManipulator::ExecuteSingleObservation() {
		DWORD observeTime = timeGetTime();
		Eigen::Vector3f posObserved;
		bool observed = m_pTracker->observe(observeTime, posObserved, m_pObject);
		if (observed && isInsideWorkspace(posObserved, m_pObject->lowerbound(), m_pObject->upperbound())) {
			m_pObject->updateStates(observeTime, posObserved);
			m_pObject->setTrackingStatus(true);
		}
		else if (observeTime - m_pObject->lastDeterminationTime > THRES_TIMEOUT_TRACK_MS)
		{
			m_pObject->setTrackingStatus(false);
		}
		if (m_logEnabled)
			m_obsLogStream << observeTime << "," << posObserved.x() << "," << posObserved.y() << "," << posObserved.z() << std::endl;
	}
	
	void MultiplexManipulator::ExecuteSingleActuation() {
		if (IsPaused()) {
			m_on_pause();
			return;
		}
		if (!m_pObject->isTracked() || !m_pObject->isInsideWorkspace())
			return;

		auto timeLoopInit = timeGetTime();
		auto forceToApply = ComputeForce(timeLoopInit, m_pObject);
		auto duties = ComputeDuty(forceToApply, m_pObject->position());
		ApplyActuation(duties);

		if (m_logEnabled) {
			m_controlLogStream << timeLoopInit << "," << m_pObject << ",";
			for (auto&& duty : duties) {
				m_controlLogStream << "," << duty;
			}
			m_controlLogStream << std::endl;
		}
	}

	void MultiplexManipulator::InitLogStream() {
		m_obsLogStream.open(m_obsLogName);
		m_obsLogStream << "sys_time,x,y,z" << std::endl;
		m_controlLogStream.open(m_controlLogName);
		m_controlLogStream << m_pObject->logCtrlHeader();
		for (int i_aupa = 0; i_aupa < m_pAupa->geometry()->numDevices(); i_aupa++) {
			m_controlLogStream << ",duty" << i_aupa;
		}
		m_controlLogStream << std::endl;
	}



	void MultiplexManipulator::setGain(
		const Eigen::Vector3f& gainP,
		const Eigen::Vector3f& gainD,
		const Eigen::Vector3f& gainI
	)
	{
		std::lock_guard<std::mutex> lock(m_mtx_gain);
		m_gainP = gainP;
		m_gainD = gainD;
		m_gainI = gainI;
	}

	std::shared_ptr<arfModelLinearBase> MultiplexManipulator::arfModel() {
		return m_arfModelPtr;
	}

	bool MultiplexManipulator::IsRunning() {
		std::shared_lock<std::shared_mutex> lk(m_mtx_isRunning);
		return m_isRunning;
	}

	bool MultiplexManipulator::IsPaused() {
		std::lock_guard<std::mutex> lock(m_mtx_isPaused);
		return m_isPaused;
	}

	void MultiplexManipulator::EnableLog(
		const std::string& obsLogName,
		const std::string& controlLogName
	) {
		if (!IsRunning()) {
			m_obsLogName = obsLogName;
			m_controlLogName = controlLogName;
			m_logEnabled = true;
		}
	}

	void MultiplexManipulator::DisableLog() {
		if (!IsRunning()) {
			m_logEnabled = false;
		}
	}

	Eigen::Vector3f MultiplexManipulator::gainP() {
		std::lock_guard<std::mutex> lock(m_mtx_gain);
		return m_gainP;
	}

	Eigen::Vector3f MultiplexManipulator::gainD() {
		std::lock_guard<std::mutex> lock(m_mtx_gain);
		return m_gainD;
	}

	Eigen::Vector3f MultiplexManipulator::gainI() {
		std::lock_guard<std::mutex> lock(m_mtx_gain);
		return m_gainI;
	}
}
