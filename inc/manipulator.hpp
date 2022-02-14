#ifndef _DYNAMAN_STRATEGY_HPP
#define _DYNAMAN_STRATEGY_HPP

#include <fstream>
#include "arfModel.hpp"
#include "FloatingObject.hpp"
#include "tracker.hpp"
#include "autd3.hpp"
#include "WinMultiplexer.hpp"

namespace dynaman {

	class Navigator {
	public:
		virtual bool Navigate(std::shared_ptr<Tracker> pTracker, FloatingObjectPtr pObject) = 0;
	};


	class Manipulator {
	public:
		Manipulator() {}
		virtual ~Manipulator() {}
		Manipulator(const Manipulator& m) = delete;
		Manipulator operator=(const Manipulator& m) = delete;
		Manipulator(Manipulator&& m) = default;
		Manipulator& operator=(Manipulator && m) = default;

		virtual int StartManipulation(
			std::shared_ptr<autd::Controller> pAupa,
			std::shared_ptr<Tracker> pTracker,
			FloatingObjectPtr pObject
		) = 0;		
		virtual void FinishManipulation() = 0;
		virtual void PauseManipulation() = 0;
		virtual void ResumeManipulation() = 0;
	};

	class MultiplexManipulator : public Manipulator {
	private:
		std::shared_ptr<autd::Controller> m_pAupa;
		std::shared_ptr<Tracker> m_pTracker;
		FloatingObjectPtr m_pObject;
		Eigen::Vector3f m_gainP;
		Eigen::Vector3f m_gainD;
		Eigen::Vector3f m_gainI;
		float m_freqLm;
		int loopPeriod_;
		float m_lambda;
		std::shared_ptr<arfModelLinearBase> m_arfModelPtr;
		bool m_isRunning;
		bool m_isPaused;
		std::string m_obsLogName;
		std::string m_controlLogName;
		bool m_logEnabled;
		std::thread m_thr_control;
		std::shared_mutex m_mtx_isRunning;
		std::mutex m_mtx_isPaused;
		std::mutex m_mtx_gain;
		std::ofstream m_obsLogStream;
		std::ofstream m_controlLogStream;

	public:
		MultiplexManipulator(
			const Eigen::Vector3f& gainP,
			const Eigen::Vector3f& gainD,
			const Eigen::Vector3f& gainI,
			float freqLm,
			int loopPeriod,
			float lambda,
			std::shared_ptr<arfModelLinearBase> arfModelPtr
		);

		MultiplexManipulator(const MultiplexManipulator& m) = delete;
		MultiplexManipulator operator=(const MultiplexManipulator& m) = delete;
		MultiplexManipulator(MultiplexManipulator&& m) = default;
		MultiplexManipulator& operator=(MultiplexManipulator && m) = default;

		static std::shared_ptr<Manipulator> Create(
			const Eigen::Vector3f& gainP,
			const Eigen::Vector3f& gainD,
			const Eigen::Vector3f& gainI,
			float freqLm = 100.f,
			int loopPeriod = 11,
			float lambda = 0.0f,
			std::shared_ptr<arfModelLinearBase> arfModelPtr = std::make_shared<arfModelFocusSphereExp50mm>()//std::make_shared<arfModelFocusOnSphereExperimental>()
		);

		int StartManipulation(
			std::shared_ptr<autd::Controller> pAupa,
			std::shared_ptr<Tracker> pTracker,
			FloatingObjectPtr pObject
		) override;

		void FinishManipulation() override;

		void PauseManipulation() override;

		void ResumeManipulation() override;

		bool IsRunning();

		bool IsPaused();

		void ExecuteSingleObservation(
			std::shared_ptr<Tracker> pTracker,
			FloatingObjectPtr pObject
		);

		void ExecuteSingleActuation(
			std::shared_ptr<autd::Controller> pAupa,
			FloatingObjectPtr pObject
		);

		Eigen::VectorXf ComputeDuty(const Eigen::Vector3f& forceTarget, const Eigen::Vector3f& position);

		Eigen::VectorXf ComputeDuty(
			const Eigen::Vector3f& forceTarget,
			const Eigen::Vector3f& position,
			size_t numAutdMax
		);

		std::vector<autd::GainPtr> CreateLateralGainList(const Eigen::VectorXf& duties, const Eigen::Vector3f& focus);

		void SetGain(const Eigen::Vector3f& gainP, const Eigen::Vector3f& gainD, const Eigen::Vector3f& gainI);

		std::shared_ptr<arfModelLinearBase> arfModel();

		void EnableLog(
			const std::string& obsLogName,
			const std::string& controlLogName
		);

		void DisableLog();
	};
}
#endif // !_DYNAMAN_STRATEGY_HPP
