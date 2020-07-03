#include <algorithm>
#include "geometryUtil.hpp"
#include "GainPlan.hpp"
#include "QPSolver.h"
#include "manipulator.hpp"

namespace dynaman {

	MultiplexManipulator::MultiplexManipulator(
		const Eigen::Vector3f& gainP,
		const Eigen::Vector3f& gainD,
		const Eigen::Vector3f& gainI,
		float freqLm,
		int loopPeriod,
		float lambda,
		std::shared_ptr<arfModelLinearBase> arfModelPtr)
		:m_gainP(gainP),
		m_gainD(gainD),
		m_gainI(gainI),
		m_freqLm(freqLm),
		m_lambda(lambda),
		m_loopPeriod(loopPeriod),
		m_arfModelPtr(arfModelPtr),
		m_isRunning(false)
	{}

	std::shared_ptr<Manipulator> MultiplexManipulator::Create(
		const Eigen::Vector3f& gainP,
		const Eigen::Vector3f& gainD,
		const Eigen::Vector3f& gainI,
		float freqLm,
		int loopPeriod,
		float lambda,
		std::shared_ptr<arfModelLinearBase> arfModelPtr
	) {
		return std::make_shared<MultiplexManipulator>(
			gainP,
			gainD,
			gainI,
			freqLm,
			loopPeriod,
			lambda,
			arfModelPtr
			);
	}

	Eigen::VectorXf MultiplexManipulator::ComputeDuty(const Eigen::Vector3f& force, const Eigen::Vector3f& position) {
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
		EigenCgalQpSolver(resultActive,
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
		return std::move(resultFull);
	}

	std::vector<autd::GainPtr> MultiplexManipulator::CreateLateralGainList(
		const Eigen::VectorXf& duties,
		const Eigen::Vector3f& focus
	) {
		std::vector<float> dutiesStl(duties.size());
		Eigen::Map<Eigen::VectorXf>(&dutiesStl[0], duties.size()) = duties;
		int num_active = (duties.array() > 1.0e-3f).count();
		std::vector<autd::GainPtr> gain_list(num_active);
		int id_begin_search = 0;
		for (auto itr_list = gain_list.begin(); itr_list != gain_list.end(); itr_list++) {
			std::map<int, autd::GainPtr> gain_map;
			auto itr_duties = std::find_if(dutiesStl.begin() + id_begin_search, dutiesStl.end(), [](float u) {return u > 0; });
			for (int i_autd = 0; i_autd < m_pAupa->geometry()->numDevices(); i_autd++) {
				if (i_autd == std::distance(dutiesStl.begin(), itr_duties)) {
					int amplitude = std::max(0, (std::min(255, static_cast<int>((*itr_duties) * 255.f * num_active))));
					//std::cout << amplitude << std::endl;
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

	void MultiplexManipulator::ExecuteSingleLoop(
		std::shared_ptr<autd::Controller> pAupa,
		std::shared_ptr<Tracker> pTracker,
		FloatingObjectPtr pObject
	) {
		DWORD timeLoopInit = timeGetTime();
		Eigen::Vector3f posObserved;
		DWORD observeTime;
		bool observed = m_pTracker->observe(observeTime, posObserved, pObject);
		if (observed && isInsideWorkspace(posObserved, pObject->lowerbound(), pObject->upperbound()))
		{
			m_pObject->updateStates(observeTime, posObserved);
			m_pObject->SetTrackingStatus(true);
			Eigen::Vector3f accel;
			{
				std::lock_guard<std::mutex> lock(m_mtx_gain);
				accel
					= m_gainP.asDiagonal() * (posObserved - pObject->getPositionTarget())
					+ m_gainD.asDiagonal() * (m_pObject->averageVelocity() - pObject->getVelocityTarget())
					+ m_gainI.asDiagonal() * m_pObject->getIntegral()
					+ m_pObject->getAccelTarget();
			}
			Eigen::Vector3f forceToApply
				= m_pObject->totalMass() * accel
				+ m_pObject->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665e3f);
			auto duties = ComputeDuty(forceToApply, posObserved);
			int num_active = (duties.array() > 1.0e-3f).count();
			if (num_active == 0) {
				m_pAupa->AppendGainSync(autd::NullGain::Create());
			}
			else {
				auto gain_list = CreateLateralGainList(duties, posObserved);
				pAupa->ResetLateralGain();
				pAupa->AppendLateralGain(gain_list);
				pAupa->StartLateralModulation(m_freqLm);
			}
		}
		else if (observeTime - pObject->lastDeterminationTime > 1000)
		{
			pObject->SetTrackingStatus(false);
		}
		int waitTime = m_loopPeriod - (timeGetTime() - timeLoopInit);
		timeBeginPeriod(1);
		Sleep(std::max(waitTime, 0));
		timeEndPeriod(1);
	}

	int MultiplexManipulator::StartManipulation(
		std::shared_ptr<autd::Controller> pAupa,
		std::shared_ptr<Tracker> pTracker,
		FloatingObjectPtr pObject
	){
		m_pAupa = pAupa;
		m_pTracker = pTracker;
		m_pObject = pObject;
		if (!pTracker->isOpen()) {
			std::cerr << "ERROR: Tracker is NOT open!" << std::endl;
			return 1;
		}
		if (!pAupa->isOpen()) {
			pAupa->Open(autd::LinkType::ETHERCAT);
			std::cerr << "ERROR: AUPA controller is not open." << std::endl;
			return 1;
		}
		{
			std::lock_guard<std::shared_mutex> lock(m_mtx_isRunning);
			m_isRunning = true;
		}
		m_thr_control = std::thread([this]()
			{
				while (this->IsRunning()) {
					this->ExecuteSingleLoop(m_pAupa, m_pTracker, m_pObject);
				}
			}
		);
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
		m_pAupa->AppendGainSync(autd::NullGain::Create());
	}

	void MultiplexManipulator::SetGain(
		const Eigen::Vector3f& gainP,
		const Eigen::Vector3f& gainD,
		const Eigen::Vector3f& gainI)
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
}