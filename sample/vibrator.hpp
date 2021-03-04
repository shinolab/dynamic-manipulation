#ifndef _DYNAMAN_VIBRATOR_HPP
#define _DYNAMAN_VIBRATOR_HPP

#include <fstream>
#include <memory>
#include <thread>
#include <random>
#include "autd3.hpp"
#include "StereoTracker.hpp"
#include "additionalGain.hpp"
#include "vibrator.hpp"

namespace dynaman {
	class Vibrator {
	private:
		std::shared_ptr<autd::Controller> m_aupa;
		std::shared_ptr<dynaman::Tracker> m_tracker;
		std::mutex m_mtxamp;
		std::mutex m_mtxRunning;
		std::thread m_thrVib;
		float m_amp = 0;
		int m_freq = 0;
		bool m_isRunning = false;
		bool m_reversed = false;
		std::string m_filename;

	public:
		Vibrator(
			const std::string& filename,
			std::shared_ptr<autd::Controller> pAupa,
			std::shared_ptr<dynaman::Tracker> pTracker
			);
		~Vibrator();
		void Start(dynaman::FloatingObjectPtr pObject);
		void Stop();
		void SetAmplitude(float amplitude);
		void SetFrequency(int freq);
		void SetReverse(bool reverse);
		static float decibel_to_amp(float db);
		static float amp_to_decibel(float amplitude);
	};
}

#endif // !_DYNAMAN_VIBRATOR_HPP
