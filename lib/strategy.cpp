#include <algorithm>
#include "odcs.hpp"
#include "geometryUtil.hpp"
#include "GainPlan.hpp"
#include "QPSolver.h"

namespace dynaman {

	MultiplexStrategy::MultiplexStrategy(
		const Eigen::Vector3f& gainP,
		const Eigen::Vector3f& gainD,
		const Eigen::Vector3f& gainI,
		float freqLm,
		unsigned int loopPeriod,
		float lambda,
		std::shared_ptr<arfModelLinearBase> arfModelPtr)
		:m_gainP(gainP),
		m_gainD(gainD),
		m_gainI(gainI),
		m_freqLm(freqLm),
		m_lambda(lambda),
		m_loopPeriod(loopPeriod),
		m_arfModelPtr(arfModelPtr)
	{}

	std::shared_ptr<Strategy> MultiplexStrategy::Create(
		const Eigen::Vector3f& gainP,
		const Eigen::Vector3f& gainD,
		const Eigen::Vector3f& gainI,
		float freqLm,
		unsigned int loopPeriod,
		float lambda,
		std::shared_ptr<arfModelLinearBase> arfModelPtr
	) {
		return std::make_shared<MultiplexStrategy>(
			gainP,
			gainD,
			gainI,
			freqLm,
			loopPeriod,
			lambda,
			arfModelPtr
			);
	}

	int MultiplexStrategy::Initialize(
		std::shared_ptr<autd::Controller> aupaPtr,
		std::shared_ptr<Tracker> trackerPtr,
		FloatingObjectPtr objPtr)
	{
		m_aupaPtr = aupaPtr;
		m_trackerPtr = trackerPtr;
		m_objPtr = objPtr;
		return 0;
	}

	Eigen::VectorXf MultiplexStrategy::ComputeDuty(const Eigen::Vector3f& force, const Eigen::Vector3f& position) {
		Eigen::MatrixXf posRel = position.replicate(1, m_aupaPtr->geometry()->numDevices()) - CentersAutd(m_aupaPtr->geometry());
		Eigen::MatrixXf directions = posRel.colwise().normalized();
		Eigen::Array<bool, -1, -1> isActive = ((directions.transpose() * force.normalized()).array() > 0.17f);
		if (isActive.count() == 0) {
			return Eigen::VectorXf::Zero(posRel.cols());
		}
		int numAupaActive = isActive.count();
		Eigen::MatrixXf posRelActive(3, isActive.count());
		std::vector<Eigen::Matrix3f> rotsAutd = RotsAutd(m_aupaPtr->geometry());
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

	std::vector<autd::GainPtr> MultiplexStrategy::CreateLateralGainList(
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
			for (int i_autd = 0; i_autd < m_aupaPtr->geometry()->numDevices(); i_autd++) {
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

	void MultiplexStrategy::Execute()
	{
		DWORD timeLoopInit = timeGetTime();
		Eigen::Vector3f posObserved;
		DWORD observeTime;
		bool observed = m_trackerPtr->observe(observeTime, posObserved, m_objPtr);
		if(observed && isInsideWorkspace(posObserved, m_objPtr->lowerbound(), m_objPtr->upperbound()))
		{
			m_objPtr->updateStates(observeTime, posObserved);
			m_objPtr->SetTrackingStatus(true);
			Eigen::Vector3f accel
				= m_gainP.asDiagonal() * (posObserved - m_objPtr->getPositionTarget())
				+ m_gainD.asDiagonal() * (m_objPtr->averageVelocity() - m_objPtr->getVelocityTarget())
				+ m_gainI.asDiagonal() * m_objPtr->getIntegral()
				+ m_objPtr->getAccelTarget();
			Eigen::Vector3f forceToApply 
				= m_objPtr->totalMass() * accel
				+ m_objPtr->AdditionalMass() * Eigen::Vector3f(0.f, 0.f, 9.80665e3f);			
			auto duties = ComputeDuty(forceToApply, posObserved);
			int num_active = (duties.array() > 1.0e-3f).count();		
			if (num_active == 0) {
				m_aupaPtr->AppendGainSync(autd::NullGain::Create());
			}
			else {
				auto gain_list = CreateLateralGainList(duties, posObserved);
				m_aupaPtr->ResetLateralGain();
				m_aupaPtr->AppendLateralGain(gain_list);
				m_aupaPtr->StartLateralModulation(m_freqLm);
			}
		}
		else if (observeTime - m_objPtr->lastDeterminationTime > 1000)
		{
			m_objPtr->SetTrackingStatus(false);
		}
		int waitTime = m_loopPeriod - (timeGetTime() - timeLoopInit);
		timeBeginPeriod(1);
		Sleep(std::max(waitTime, 0));
		timeEndPeriod(1);
	}

	void MultiplexStrategy::SetGain(
		const Eigen::Vector3f& gainP,
		const Eigen::Vector3f& gainD,
		const Eigen::Vector3f& gainI)
	{
		m_gainP = gainP;
		m_gainD = gainD;
		m_gainI = gainI;
	}

	std::shared_ptr<arfModelLinearBase> MultiplexStrategy::arfModel() {
		return m_arfModelPtr;
	}
}