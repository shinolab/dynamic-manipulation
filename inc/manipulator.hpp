#pragma once

#include <fstream>
#include "arfModel.hpp"
#include "FloatingObject.hpp"
#include "tracker.hpp"
#include "autd3.hpp"

namespace dynaman {

	class Manipulator {
	public:
		Manipulator() {}
		virtual ~Manipulator() {}
		Manipulator(const Manipulator& m) = delete;
		Manipulator operator=(const Manipulator& m) = delete;
		Manipulator(Manipulator&& m) = default;
		Manipulator& operator=(Manipulator && m) = default;

		virtual int StartManipulation(FloatingObjectPtr pObject) = 0;
		virtual void FinishManipulation() = 0;
		virtual void PauseManipulation() = 0;
		virtual void ResumeManipulation() = 0;
		virtual void SetOnPause(std::function<void()>& func_on_pause) = 0;
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
		int m_period_control_ms;
		int m_period_track_ms;
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
		std::function<void()> m_on_pause = []() {return; };

	public:
		MultiplexManipulator(
			std::shared_ptr<autd::Controller> pAupa,
			std::shared_ptr<dynaman::Tracker> pTracker,
			const Eigen::Vector3f& gainP = Eigen::Vector3f::Constant(32.0f),
			const Eigen::Vector3f& gainD = Eigen::Vector3f::Constant(20.0f),
			const Eigen::Vector3f& gainI = Eigen::Vector3f::Constant(0.00f),
			float freqLm = 100.f,
			int period_control_ms = 10,
			int period_track_ms = 5,
			float lambda = 0.0f,
			std::shared_ptr<arfModelLinearBase> arfModelPtr = std::make_shared<arfModelFocusSphereR50>()
		);

		MultiplexManipulator(const MultiplexManipulator& m) = delete;
		MultiplexManipulator operator=(const MultiplexManipulator& m) = delete;
		MultiplexManipulator(MultiplexManipulator&& m) = default;
		MultiplexManipulator& operator=(MultiplexManipulator && m) = default;

		static std::shared_ptr<Manipulator> Create(
			std::shared_ptr<autd::Controller> pAupa,
			std::shared_ptr<dynaman::Tracker> pTracker,
			const Eigen::Vector3f& gainP = Eigen::Vector3f::Constant(32.0f),
			const Eigen::Vector3f& gainD = Eigen::Vector3f::Constant(20.0f),
			const Eigen::Vector3f& gainI = Eigen::Vector3f::Constant(0.00f),
			float freqLm = 100.f,
			int period_control_ms = 10,
			int period_track_ms = 5,
			float lambda = 0.0f,
			std::shared_ptr<arfModelLinearBase> arfModelPtr = std::make_shared<arfModelFocusSphereR50>()
		);

		int StartManipulation(FloatingObjectPtr pObject) override;

		void FinishManipulation() override;

		void PauseManipulation() override;

		void ResumeManipulation() override;

		void SetOnPause(std::function<void()>& func_on_pause) override;

		bool IsRunning();

		bool IsPaused();

		void ExecuteSingleObservation();

		void ExecuteSingleActuation();

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

		Eigen::Vector3f gainP();

		Eigen::Vector3f gainD();

		Eigen::Vector3f gainI();
	};
}
