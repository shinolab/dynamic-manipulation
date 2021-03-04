#include "vibrator.hpp"

namespace dynaman{
	Vibrator::Vibrator(
		const std::string& filename,
		std::shared_ptr<autd::Controller> pAupa,
		std::shared_ptr<dynaman::Tracker> pTracker
		) :
		m_filename(filename),
		m_aupa(pAupa),
		m_tracker(pTracker)
	{
		if (!pAupa->isOpen()) {
			m_aupa->Open(autd::LinkType::ETHERCAT);
		}
		if (!pTracker->isOpen()) {
			m_tracker->open();
		}
	}

	Vibrator::~Vibrator() {
		m_aupa->Close();
	}

	void Vibrator::Start(dynaman::FloatingObjectPtr pObject) {
		m_thrVib = std::thread([this, &pObject]()
			{
				std::ofstream ofs(m_filename + "_pos.csv", std::ios_base::app);
				{
					std::lock_guard<std::mutex> lock(m_mtxRunning);
					m_isRunning = true;
				}
				//m_aupa->AppendModulationSync(autd::SineModulation::Create(200));
				while (m_isRunning) {
					DWORD tObserve;
					Eigen::Vector3f pos;
					bool observed = m_tracker->observe(tObserve, pos, pObject);
					if (observed) {
						std::lock_guard<std::mutex> lock(m_mtxamp);
						autd::GainPtr gain;
						if (m_reversed) {
							gain = autd::ReversibleFocalPointGain::Create(pos, 255, [](const Eigen::Vector3f& pos)
								{
									return pos.x() > 0 ? false : true;
								}
							);
						}
						else {
							gain = autd::FocalPointGain::Create(pos, 255);
						}
						m_aupa->AppendGainSync(gain);
						m_aupa->AppendModulationSync(autd::SineModulation::Create(m_freq, m_amp, 0.5f * m_amp));
						ofs << tObserve << "," << pos.x() << "," << pos.y() << "," << pos.z() << std::endl;
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(30));
				}
				ofs.close();
				m_aupa->AppendGainSync(autd::NullGain::Create());
			}
		);
	}


	void Vibrator::Stop() {
		{
			std::lock_guard<std::mutex> lock(m_mtxRunning);
			m_isRunning = false;
		}
		if (m_thrVib.joinable()) {
			m_thrVib.join();
		}
	}


	void Vibrator::SetAmplitude(float amplitude) {
		std::lock_guard<std::mutex> lock(m_mtxamp);
		m_amp = amplitude;
	}

	void Vibrator::SetFrequency(int freq) {
		std::lock_guard<std::mutex> lock(m_mtxamp);
		m_freq = freq;
	}

	void Vibrator::SetReverse(bool reverse) {
		std::lock_guard<std::mutex> lock(m_mtxamp);
		m_reversed = reverse;
	}

	float Vibrator::decibel_to_amp(float db) {
		return pow(10.0f, db / 15);
	}

	float Vibrator::amp_to_decibel(float amplitude) {
		return 15.0f * log10f(amplitude);
	}
}

