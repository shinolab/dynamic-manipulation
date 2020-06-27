#ifndef _DYNAMAN_TRAJECTORY_HPP
#define _DYNAMAN_TRAJECTORY_HPP

#include <Eigen/Geometry>
#define NOMINMAX
#include <Windows.h>
#pragma comment (lib, "winmm")

namespace dynaman {
	class Trajectory
	{
	public:
		virtual ~Trajectory() {};
		virtual Eigen::Vector3f pos(DWORD time_ms = timeGetTime()) = 0;
		virtual Eigen::Vector3f vel(DWORD time_ms = timeGetTime()) = 0;
		virtual Eigen::Vector3f accel(DWORD time_ms = timeGetTime()) = 0;
	};

	class TrajectoryConstantState : public Trajectory
	{
	private:
		Eigen::Vector3f posTgt;
		Eigen::Vector3f velTgt;
		Eigen::Vector3f accelTgt;
	public:
		TrajectoryConstantState(Eigen::Vector3f const& positionTarget,
			Eigen::Vector3f const& velocityTarget = Eigen::Vector3f::Constant(0.f),
			Eigen::Vector3f const& accelTarget = Eigen::Vector3f::Constant(0.f));
		Eigen::Vector3f pos(DWORD sys_time_ms = timeGetTime()) override;
		Eigen::Vector3f vel(DWORD sys_time_ms = timeGetTime()) override;
		Eigen::Vector3f accel(DWORD sys_time_ms = timeGetTime()) override;
		static std::shared_ptr<Trajectory> Create(Eigen::Vector3f const& positionTarget,
			Eigen::Vector3f const& velocityTarget,
			Eigen::Vector3f const& accelTarget);
	};

	class TrajectoryBangBang : public Trajectory
	{
	private:
		float _timeTotal;
		DWORD _sys_time_init;
		Eigen::Vector3f _posInit;
		Eigen::Vector3f _posEnd;
		Eigen::Vector3f _velInit;
		Eigen::Vector3f _velEnd;

	public:
		TrajectoryBangBang(float timeTotal,
			DWORD sys_time_init,
			Eigen::Vector3f const& posInit,
			Eigen::Vector3f const& posEnd)
			:_timeTotal(timeTotal),
			_sys_time_init(sys_time_init),
			_posInit(posInit),
			_posEnd(posEnd) {}

		Eigen::Vector3f pos(DWORD time_ms) override;
		Eigen::Vector3f vel(DWORD time_ms) override;
		Eigen::Vector3f accel(DWORD time_ms) override;
		static std::shared_ptr<Trajectory> Create(float timeTotal,
			DWORD sys_time_init_ms,
			Eigen::Vector3f const& posInit,
			Eigen::Vector3f const& posEnd);
	};

	class TrajectoryBang : public Trajectory {
	private:
	private:
		float timeToGo;
		float timeInit;
		Eigen::Vector3f posInit;
		Eigen::Vector3f posEnd;
	public:
		TrajectoryBang(float const& timeToGo,
			float const& timeInit,
			Eigen::Vector3f const& posInit,
			Eigen::Vector3f const& posEnd)
		{
			this->timeToGo = timeToGo;
			this->timeInit = timeInit;
			this->posInit = posInit;
			this->posEnd = posEnd;
		}
		Eigen::Vector3f pos(DWORD const time = timeGetTime()) override;
		Eigen::Vector3f vel(DWORD const time = timeGetTime()) override;
		Eigen::Vector3f accel(DWORD const time = timeGetTime()) override;
		static std::shared_ptr<Trajectory> Create(float const& timeToGo,
			float const& timeInit,
			Eigen::Vector3f const& posInit,
			Eigen::Vector3f const& posEnd);
	};

	class TrajectoryCircle : public Trajectory {
		Eigen::Vector3f _center;
		float _radius;
		float _inclination;
		float _raan;
		float _omega;
		float _phaseInit;
		float _sys_time_init;

	public:
		TrajectoryCircle(
			const Eigen::Vector3f& _center,
			float _radius,
			float _inclination,
			float _raan,
			float _period,
			float _phaseInit,
			DWORD _sys_time_init
		);
		static std::shared_ptr<Trajectory> Create(
			const Eigen::Vector3f& center,
			float radius,
			float inclination,
			float raan,
			float period_sec,
			float phaseInit,
			DWORD sys_time_init_ms
		);
		Eigen::Vector3f pos(DWORD sys_time_ms = timeGetTime()) override;
		Eigen::Vector3f vel(DWORD sys_time_ms = timeGetTime()) override;
		Eigen::Vector3f accel(DWORD sys_time_ms = timeGetTime()) override;
		float Radius();
		float Phase(DWORD sys_time_ms);
	};

	class TrajectorySinusoid : public Trajectory {
		Eigen::Vector3f _direction;
		float _amplitude;
		float _period;
		Eigen::Vector3f _center;
		DWORD _sys_time_init;

	public:
		TrajectorySinusoid(
			const Eigen::Vector3f& direction,
			float amplitude,
			float period,
			const Eigen::Vector3f& center,
			DWORD sys_time_init
		);

		static std::shared_ptr<Trajectory> Create(
			const Eigen::Vector3f& direction,
			float amplitude,
			float period,
			const Eigen::Vector3f& center,
			DWORD sys_time_init
		);

		Eigen::Vector3f pos(DWORD time = timeGetTime()) override;
		Eigen::Vector3f vel(DWORD time = timeGetTime()) override;
		Eigen::Vector3f accel(DWORD time = timeGetTime()) override;
		float Phase(DWORD sys_time);
		float Omega(DWORD sys_time);
	};

	class TrajectoryInfShape : public Trajectory {
		Eigen::Vector3f _center;
		float _height;
		float _width;
		float _omega;
		DWORD _sys_time_init;
	public:
		TrajectoryInfShape(
			const Eigen::Vector3f& center,
			float height,
			float width,
			float period_sec,
			DWORD sys_time_init_ms
		);

		~TrajectoryInfShape() = default;

		static std::shared_ptr<Trajectory> Create(
			const Eigen::Vector3f& center,
			float height,
			float width,
			float period_sec,
			DWORD sys_time_init_ms
		);

		Eigen::Vector3f pos(DWORD sys_time_ms) override;

		Eigen::Vector3f vel(DWORD sys_time_ms) override;

		Eigen::Vector3f accel(DWORD sys_time_ms) override;

		float Phase(DWORD sys_time_ms);
	};

	class TrajectoryHeart : public Trajectory {
		Eigen::Vector3f _center;
		float _height;
		float _width;
		float _omega;
		DWORD _sys_time_init;
	public:
		TrajectoryHeart(
			const Eigen::Vector3f& center,
			float height,
			float width,
			float period_sec,
			DWORD sys_time_init_ms
		);

		static std::shared_ptr<Trajectory> Create(
			const Eigen::Vector3f& center,
			float height,
			float width,
			float period_sec,
			DWORD sys_time_init_ms
		);

		~TrajectoryHeart() = default;

		float Phase(DWORD sys_time_ms);

		Eigen::Vector3f pos(DWORD sys_time_ms) override;

		Eigen::Vector3f vel(DWORD sys_time_ms) override;

		Eigen::Vector3f accel(DWORD sys_time_ms)override;
	};
}


#endif