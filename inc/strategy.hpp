#ifndef _DYNAMAN_STRATEGY_HPP
#define _DYNAMAN_STRATEGY_HPP

#include "arfModel.hpp"
#include "FloatingObject.hpp"
#include "tracker.hpp"
#include "autd3.hpp"

namespace dynaman {
	class Strategy {
	protected:
		std::shared_ptr<autd::Controller> m_aupaPtr;
		std::shared_ptr<Tracker> m_trackerPtr;
		FloatingObjectPtr m_objPtr;
	public:
		virtual ~Strategy() {}
		virtual int Initialize(
			std::shared_ptr<autd::Controller> aupa,
			std::shared_ptr<Tracker> tracker,
			FloatingObjectPtr objPtr) = 0;
		virtual void Execute() = 0;
	};

	class MultiplexStrategy : public Strategy {
	private:
		Eigen::Vector3f m_gainP;
		Eigen::Vector3f m_gainD;
		Eigen::Vector3f m_gainI;
		unsigned int m_freqLm;
		float m_lambda;
		std::shared_ptr<arfModelLinearBase> m_arfModelPtr;

	public:
		MultiplexStrategy(
			const Eigen::Vector3f& gainP,
			const Eigen::Vector3f& gainD,
			const Eigen::Vector3f& gainI,
			unsigned int freqLm,
			float lambda,
			std::shared_ptr<arfModelLinearBase> arfModelPtr
		);

		static std::shared_ptr<Strategy> Create(
			const Eigen::Vector3f& gainP,
			const Eigen::Vector3f& gainD,
			const Eigen::Vector3f& gainI,
			unsigned int freqLm = 100,
			float lambda = 0.f,
			std::shared_ptr<arfModelLinearBase> arfModelPtr = std::make_shared<arfModelFocusOnSphereExperimental>()
		);

		int Initialize(
			std::shared_ptr<autd::Controller> aupaPtr,
			std::shared_ptr<Tracker> trackerPtr,
			FloatingObjectPtr objPtr) override;

		void Execute() override;

		Eigen::VectorXf ComputeDuty(const Eigen::Vector3f& forceTarget, const Eigen::Vector3f& position);

		std::vector<autd::GainPtr> CreateLateralGain(const Eigen::VectorXf& duties, const Eigen::Vector3f& focus);

		void SetGain(const Eigen::Vector3f& gainP, const Eigen::Vector3f& gainD, const Eigen::Vector3f& gainI);
	};
}
#endif // !_DYNAMAN_STRATEGY_HPP