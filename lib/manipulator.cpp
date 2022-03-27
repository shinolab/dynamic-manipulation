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

	bool IsInitialized(
		std::shared_ptr<autd::Controller> pAupa,
		std::shared_ptr<Tracker> pTracker
	) {
		if (!pTracker->isOpen()) {
			std::cerr << "ERROR: Tracker is NOT open." << std::endl;
			return false;
		}
		if (!pAupa->isOpen()) {
			std::cerr << "ERROR: AUPA controller is not open." << std::endl;
			return false;
		}
		return true;
	}

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

	Eigen::VectorXf MultiplexManipulator::ComputeDuty(
		const Eigen::Vector3f& force,
		const Eigen::Vector3f& position
	) {
		Eigen::MatrixXf posRel = position.replicate(1, m_pAupa->geometry()->numDevices()) - CentersAutd(m_pAupa->geometry());
		Eigen::MatrixXf directions = posRel.colwise().normalized();
		Eigen::Array<bool, -1, -1> isActive = ((directions.transpose() * force.normalized()).array() > 0.17f);
		if (isActive.count() == 0) {
			return Eigen::VectorXf::Zero(posRel.cols());
		}
		int numAupaActive = isActive.count();
		Eigen::MatrixXf posRelActive(3, isActive.count());
		std::vector<Eigen::Matrix3f> rotsAutd = RotsAutd(m_pAupa->geometry());
		std::vector<Eigen::Matrix3f> rotsAutdActive(isActive.count());
		int iAupaActive = 0;
		for (int iAupa = 0; iAupa < posRel.cols(); iAupa++) {
			if (isActive(iAupa)) {
				posRelActive.col(iAupaActive) << posRel.col(iAupa);
				rotsAutdActive[iAupaActive] = rotsAutd[iAupa];
				iAupaActive++;
			}
		}
		Eigen::MatrixXf F = m_arfModelPtr->arf(posRelActive, rotsAutdActive);
		Eigen::VectorXi condEq(1);
		condEq << -1;
		Eigen::MatrixXf A(1, numAupaActive);
		A << Eigen::RowVectorXf::Ones(numAupaActive);
		Eigen::VectorXf b(1);
		b << 1;
		Eigen::VectorXf resultActive;
		SolveQuadraticProgram(
			resultActive,
			A,
			b,
			F.transpose() * F + m_lambda * Eigen::MatrixXf::Identity(numAupaActive, numAupaActive),
			-F.transpose() * force,
			condEq,
			Eigen::VectorXf::Zero(numAupaActive),
			Eigen::VectorXf::Ones(numAupaActive)
		);
		iAupaActive = 0;
		Eigen::VectorXf resultFull(posRel.cols());
		for (int iAupa = 0; iAupa < posRel.cols(); iAupa++) {
			if (isActive(iAupa)) {
				resultFull(iAupa) = resultActive(iAupaActive);
				iAupaActive++;
			}
			else {
				resultFull(iAupa) = 0;
			}
		}
		return resultFull;
	}

	Eigen::VectorXf MultiplexManipulator::ComputeDuty(
		const Eigen::Vector3f& force,
		const Eigen::Vector3f& pos,
		size_t numAupaMax
	) {
		Eigen::MatrixXf posRelAll = pos.replicate(1, m_pAupa->geometry()->numDevices()) - CentersAutd(m_pAupa->geometry());
		auto rotsAutdAll = RotsAutd(m_pAupa);
		Eigen::MatrixXf fMatAll = m_arfModelPtr->arf(posRelAll, rotsAutdAll);

		Eigen::VectorXf alignment_eigen = fMatAll.colwise().normalized().transpose() * force.normalized();
		std::vector<float> alignment(m_pAupa->geometry()->numDevices());
		Eigen::Map<Eigen::VectorXf>(&alignment[0], alignment.size()) = alignment_eigen; // [x1, x2, ... xN]
		auto alignment_sorted(alignment); std::sort(alignment_sorted.begin(), alignment_sorted.end(), std::greater<float>());

		std::vector<int> deviceIdUsed(numAupaMax, -1);
		for (int i = 0; i < deviceIdUsed.size(); i++) {
			auto search_begin = alignment.begin();
			while (true) {
				auto itr = std::find(search_begin, alignment.end(), alignment_sorted[i]);
				auto deviceId = std::distance(alignment.begin(), itr);
				if (std::find(deviceIdUsed.begin(), deviceIdUsed.end(), deviceId) == deviceIdUsed.end()) {
					deviceIdUsed[i] = deviceId;
					break;
				}
				else {
					search_begin = ++itr;
				}
			}
		}

		std::vector<std::vector<size_t>> combinations;
		for (size_t numAupa = 1; numAupa <= numAupaMax; numAupa++) {
			auto temp = combination(numAupaMax - 1, numAupa);
			combinations.insert(combinations.end(), temp.begin(), temp.end());
		}

		std::vector<std::pair<float, std::vector<float>>> resAndDuties(combinations.size());
		std::vector<std::future<void>> fut;
		for (int ic = 0; ic < combinations.size(); ic++) {
			fut.push_back(
				std::async(
					std::launch::async,
					[&combinations, &force, &posRelAll, &rotsAutdAll, &deviceIdUsed, ic, &resAndDuties, this]() {
						auto comb = combinations[ic];

						Eigen::MatrixXf posRelUsed(3, comb.size());
						std::vector<Eigen::Matrix3f> rotsAutdUsed;
						for (int iCol = 0; iCol < comb.size(); iCol++) {
							posRelUsed.col(iCol) = posRelAll.col(deviceIdUsed[comb[iCol]]);
							rotsAutdUsed.push_back(rotsAutdAll[deviceIdUsed[comb[iCol]]]);
						}
						Eigen::MatrixXf A = Eigen::MatrixXf::Ones(1, comb.size());
						Eigen::VectorXf b = Eigen::VectorXf::Ones(1);
						Eigen::MatrixXf F = m_arfModelPtr->arf(posRelUsed, rotsAutdUsed);
						Eigen::MatrixXf D = F.transpose() * F + m_lambda * Eigen::MatrixXf::Identity(comb.size(), comb.size());
						Eigen::VectorXf c = -F.transpose() * force;
						Eigen::VectorXi condEq = Eigen::VectorXi::Constant(1, -1);
						Eigen::VectorXf duty_min = Eigen::VectorXf::Zero(comb.size());
						Eigen::VectorXf duty_max = Eigen::VectorXf::Constant(comb.size(), 1.0f / comb.size());
						Eigen::VectorXf resultPart;

						SolveQuadraticProgram(resultPart, A, b, D, c, condEq, duty_min, duty_max);
						float res = (F * resultPart - force).norm();
						std::vector<float> duty_ordered(m_pAupa->geometry()->numDevices(), 0.f);
						for (int id = 0; id < c.size(); id++) {
							duty_ordered[deviceIdUsed[comb[id]]] = resultPart[id];
						}
						resAndDuties[ic] = std::make_pair(res, duty_ordered);
					}
				)
			);
		}
		for (auto&& f : fut) {
			f.get();
		}
		
		auto resAndDutyBest = std::min_element(
			resAndDuties.begin(),
			resAndDuties.end(),
			[](std::pair<float, std::vector<float>> smaller, std::pair<float, std::vector<float>> larger) {
				return smaller.first < larger.first; 
			}
		);
		Eigen::VectorXf duty_best_eigen(m_pAupa->geometry()->numDevices());
		for (int i = 0; i < resAndDutyBest->second.size(); i++) {
			duty_best_eigen[i] = resAndDutyBest->second[i];
		}
		return duty_best_eigen;
	}

	void MultiplexManipulator::SetOnPause(std::function<void()>& func_on_pause) {
		m_on_pause = func_on_pause;
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
					int amplitude = std::max(0, (std::min(255, static_cast<int>((*itr_duties) * 255.f * num_active))));
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
		if (m_logEnabled) {
			m_obsLogStream << observeTime << "," << posObserved.x() << "," << posObserved.y() << "," << posObserved.z() << std::endl;
		}
	}
	

	void MultiplexManipulator::ExecuteSingleActuation() {
		if (IsPaused()) {
			m_on_pause();
			return;
		}
		auto timeLoopInit = timeGetTime();
		Eigen::Vector3f pos, vel, integ;
		m_pObject->getStates(pos, vel, integ);
		auto posTgt = m_pObject->getPositionTarget(timeLoopInit);
		auto velTgt = m_pObject->getVelocityTarget(timeLoopInit);
		auto accelTgt = m_pObject->getAccelTarget(timeLoopInit);

		if (m_pObject->isTracked() && m_pObject->isInsideWorkspace())
		{
			Eigen::Vector3f accel;
			{
				std::lock_guard<std::mutex> lock(m_mtx_gain);
				accel
					= m_gainP.asDiagonal() * (posTgt - pos)
					+ m_gainD.asDiagonal() * (velTgt - vel)
					- m_gainI.asDiagonal() * integ
					+ accelTgt;
			}
			Eigen::Vector3f forceToApply
				= m_pObject->totalMass() * accel
				+ m_pObject->weight() * Eigen::Vector3f(0.f, 0.f, GRAVITY_ACCEL);
			auto duties = ComputeDuty(forceToApply, pos);
			int num_active = (duties.array() > 1.0e-3f).count();
			if (num_active == 0) {
				m_pAupa->AppendGainSync(autd::NullGain::Create());
			}
			else {
				auto gain_list = CreateLateralGainList(duties, pos);
				m_pAupa->ResetLateralGain();
				m_pAupa->AppendLateralGain(gain_list);
				m_pAupa->StartLateralModulation(m_freqLm);
			}
			if (m_logEnabled) {
				m_controlLogStream
					<< timeLoopInit << ","
					<< pos.x() << "," << pos.y() << "," << pos.z() << ","
					<< vel.x() << "," << vel.y() << "," << vel.z() << ","
					<< integ.x() << "," << integ.y() << "," << integ.z() << ","
					<< posTgt.x() << "," << posTgt.y() << "," << posTgt.z() << ","
					<< velTgt.x() << "," << velTgt.y() << "," << velTgt.z() << ","
					<< accelTgt.x() << "," << accelTgt.y() << "," << accelTgt.z() << ","
					<< accel.x() << "," << accel.y() << "," << accel.z() << ",";
				Eigen::MatrixXf posRel = pos.replicate(1, m_pAupa->geometry()->numDevices()) - CentersAutd(m_pAupa->geometry());
				Eigen::Vector3f forceResult = m_arfModelPtr->arf(posRel, RotsAutd(m_pAupa->geometry())) * duties;
				m_controlLogStream << forceResult.x() << "," << forceResult.y() << "," << forceResult.z();
				for (int i_duty = 0; i_duty < duties.size(); i_duty++) {
					m_controlLogStream << "," << duties[i_duty];
				}
				m_controlLogStream << std::endl;
			}
		}
	}

	int MultiplexManipulator::StartManipulation(
		FloatingObjectPtr pObject
	){
		m_pObject = pObject;
		if (!m_pTracker->isOpen()) {
			std::cerr << "ERROR: Tracker is NOT open!" << std::endl;
			return -1;
		}
		if (!m_pAupa->isOpen()) {
			std::cerr << "ERROR: AUPA controller is not open." << std::endl;
			return -1;
		}
		if (m_logEnabled) {
			m_obsLogStream.open(m_obsLogName);
			m_obsLogStream << "sys_time,x,y,z" << std::endl;
			m_controlLogStream.open(m_controlLogName);
			m_controlLogStream << "sys_time,x,y,z,vx,vy,vz,ix,iy,iz,xTgt,yTgt,zTgt,vxTgt,vyTgt,vzTgt,axTgt,ayTgt,azTgt,axRes,ayRes,azRes,Fxres,Fyres,Fzres";
			for (int i_aupa = 0; i_aupa < m_pAupa->geometry()->numDevices(); i_aupa++) {
				m_controlLogStream << ",duty" << i_aupa;
			}
			m_controlLogStream << std::endl;
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

	void MultiplexManipulator::SetGain(
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
