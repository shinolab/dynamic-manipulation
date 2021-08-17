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

	class SimpleNavigator : Navigator {
		std::ofstream ofs;
	public:
		void InitLog(const std::string& logName);
		void CloseLog();
		bool Navigate(std::shared_ptr<Tracker> pTracker, FloatingObjectPtr pObject) override;
	};

	class Manipulator {
	public:
		virtual ~Manipulator() {}
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
		int m_loopPeriodAupa;
		int m_loopPeriodTracker;
		float m_lambda;
		std::shared_ptr<arfModelLinearBase> m_arfModelPtr;
		bool m_isRunning;
		bool m_isPaused;
		std::string m_obsLogName;
		std::string m_controlLogName;
		bool m_logEnabled;
		std::thread m_thr_control;
		std::thread m_thr_track;
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
			int loopPeriodAupa,
			int loopPeriodTracker,
			float lambda,
			std::shared_ptr<arfModelLinearBase> arfModelPtr
		);

		static std::shared_ptr<Manipulator> Create(
			const Eigen::Vector3f& gainP,
			const Eigen::Vector3f& gainD,
			const Eigen::Vector3f& gainI,
			float freqLm = 100.f,
			int loopPeriodAupa = 10,
			int loopPeriodTracker = 5,
			float lambda = 0.f,
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

		std::vector<autd::GainPtr> CreateLateralGainList(const Eigen::VectorXf& duties, const Eigen::Vector3f& focus);

		void SetGain(const Eigen::Vector3f& gainP, const Eigen::Vector3f& gainD, const Eigen::Vector3f& gainI);

		std::shared_ptr<arfModelLinearBase> arfModel();

		void EnableLog(
			const std::string& obsLogName,
			const std::string& controlLogName
		);
		void DisableLog();
	};

	class VarMultiplexManipulator: public Manipulator {
	private:
		SimpleNavigator navigator;
		Eigen::Vector3f m_gainP;
		Eigen::Vector3f m_gainD;
		Eigen::Vector3f m_gainI;
		int m_periodMux_us;
		int m_periodControl_ms;
		int m_periodObs_ms;
		float m_lambda;
		float m_duty_max;
		std::shared_ptr<arfModelLinearBase> m_arfModelPtr;
		bool m_isRunning;
		bool m_isPaused;
		bool m_logEnabled;
		std::thread m_thr_control;
		std::thread m_thr_track;
		std::shared_mutex m_mtx_isRunning;
		std::mutex m_mtx_isPaused;
		std::mutex m_mtx_gain;
		std::string m_controlLogName;
		std::ofstream m_controlLogStream;
	public:
		std::shared_ptr<autd::Controller> m_pAupa;
		std::shared_ptr<Tracker> m_pTracker;
		FloatingObjectPtr m_pObject;
		WinMultiplexer mux;

		VarMultiplexManipulator(
			const Eigen::Vector3f& gainP,
			const Eigen::Vector3f& gainD,
			const Eigen::Vector3f& gainI,
			int periodMux_us,
			int periodUpdateControl_ms,
			int periodUpdateObs_ms,
			float duty_max,
			float lambda,
			std::shared_ptr<arfModelLinearBase> arfModelPtr
		);

		static std::shared_ptr<Manipulator> Create(
			const Eigen::Vector3f& m_gainP,
			const Eigen::Vector3f& m_gainD,
			const Eigen::Vector3f& m_gainI,
			int periodMux_us = 10000,
			int updatePeriodAupa_ms = 10,
			int updatePeriodTraceker_ms = 5,
			float duty_max = 1.0,
			float lambda = 0.0f,
			std::shared_ptr<arfModelLinearBase> arfModelPtr = std::make_shared<arfModelFocusSphereExp50mm>()
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

		void ExecuteSingleActuation(
			std::shared_ptr<autd::Controller> pAupa,
			FloatingObjectPtr pObject
		);

		void ExecuteOnPaused(
			std::shared_ptr<autd::Controller> pAupa,
			FloatingObjectPtr pObject
		);

		Eigen::VectorXf ComputeDuty(const Eigen::Vector3f& forceTarget, const Eigen::Vector3f& focus);

		std::vector<std::pair<autd::GainPtr, int>> CreateDriveSequence(const Eigen::VectorXf& duties, const Eigen::Vector3f& focus);

		std::vector<std::pair<autd::GainPtr, int>> CreateDriveSequenceOld(const Eigen::VectorXf& duties, const Eigen::Vector3f& focus);

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
