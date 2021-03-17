#include <iostream>
#include <fstream>
#include <memory>
#include <thread>
#include <chrono>
#include <random>
#include "autd3.hpp"
#include "manipulator.hpp"
#include "haptic_icon.hpp"
#include "vibrator.hpp"

using namespace dynaman;

enum class Series{
	ASCEND = 0,
	DESCEND = 1
};

enum class Answer {
	EVAL_WEAK = 0,
	EVAL_STRONG = 1,
	SAME =2
};

std::string answer_to_string(Answer answer) {
	std::string res;
	switch (answer)
	{
	case Answer::EVAL_WEAK:
		res = "EVAL_WEAK";
		break;
	case Answer::EVAL_STRONG:
		res = "EVAL_STRONG";
		break;
	case Answer::SAME:
		res = "SAME";
		break;
	default:
		break;
	}
	return res;
}

std::string series_to_string(Series series) {
	std::string res;
	switch (series)
	{
	case Series::ASCEND:
		res = "A";
		break;
	case Series::DESCEND:
		res = "D";
		break;
	default:
		res = "?";
		break;
	}
	return res;
}

bool is_final(Answer answer, Series series) {
	return (series == Series::ASCEND && answer == Answer::EVAL_STRONG) || (series == Series::DESCEND && answer == Answer::EVAL_WEAK);
}

bool ask(int idx_tgt, int idx_max) {
	std::cout << "Ask which one is the target ? [";
	for (int i = 0; i < idx_max; i++) {
		std::cout << i+1 << (i == idx_max - 1 ? "" : "/");
	}
	std::cout << "]" << std::endl;
	std::string key;
	std::cin >> key;
	return std::atoi(key.c_str())-1 == idx_tgt;
}

void present_stimulation(
	Vibrator& vibrator,
	FloatingObjectPtr pObject,
	float amp_ref,
	float amp_tgt,
	int idx_tgt,
	int num_stimulus=3,
	int freq=200,
	int duration_ms=800 
)
{
	if (idx_tgt < 0 || idx_tgt > num_stimulus - 1) {
		std::cout << "Error in Vibrator: Invalid paramter." << std::endl;
	}
	std::cout
		<< "ref/tgt amp are: " 
		<< amp_ref << " (" << Vibrator::amp_to_decibel(amp_ref) << " dB)"
		<< " / "
		<< amp_tgt << " (" << Vibrator::amp_to_decibel(amp_tgt) << " dB) "
		<< std::endl;
	Beep(440, 500);
	vibrator.SetFrequency(freq);
	for (int i = 0; i < num_stimulus; i++) {
		vibrator.SetAmplitude(i == idx_tgt ? amp_tgt : amp_ref);
		vibrator.Start(pObject);
		std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
		vibrator.Stop();
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
	Beep(520, 400);
}

float change_db(float& db_eval, float db_step, Series series) {
	if (series == Series::ASCEND) {
		db_eval += db_step;
	}
	else if(series == Series::DESCEND) {
		db_eval -= db_step;
	}
	return db_eval;
}

float run_series(
	Vibrator& vibrator,
	float db_ref,
	float db_init,
	float db_min,
	float db_max,
	float db_step,
	int freq,
	int duration_ms,
	int num_stimulus,
	FloatingObjectPtr pObject,
	Series series,
	std::ofstream& ofs
) {
	ofs << series_to_string(series) << "," << std::endl;
	float db_end = (series == Series::ASCEND ? db_max : db_min);

	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());
	std::uniform_int_distribution<> distribution(0, num_stimulus - 1);
	float db_tgt = db_init;
	for (; (db_tgt - db_init) * (db_tgt - db_end) <= 0; change_db(db_tgt, db_step, series)){
		bool detect = false;
		int i_trial = 0;
		for (; i_trial < num_stimulus;) {
			float amp_ref = Vibrator::decibel_to_amp(db_ref);
			float amp_tgt = Vibrator::decibel_to_amp(db_tgt);
			int idx_tgt = distribution(engine);
			std::cout << "idx_tgt: " << idx_tgt << std::endl;
			present_stimulation(
				vibrator,
				pObject,
				amp_ref,
				amp_tgt,
				idx_tgt,
				num_stimulus,
				freq,
				duration_ms
			);
			bool is_correct = ask(idx_tgt, num_stimulus);
			ofs << amp_ref << "," "(" << Vibrator::amp_to_decibel(amp_ref) <<"dB)," << amp_tgt << ",(" << Vibrator::amp_to_decibel(amp_tgt) << "dB)" << (is_correct ? "C" : "W") << "," << std::endl;
			if (!is_correct) {
				break;
			}
			i_trial++;
		}
		//endinig condition
		if (
			(series == Series::ASCEND && i_trial == num_stimulus) 
			|| (series == Series::DESCEND && i_trial < num_stimulus)
			) {
			std::cout << "itrial: " << i_trial << std::endl;
			break;
		}
		if ((series == Series::ASCEND && db_tgt >= db_end)
			|| (series == Series::DESCEND && db_tgt <= db_end)) {
			std::cout << "Failed to detect the difference." << std::endl;
		}
	}
	std::cout << "This series has finished. Reversing..." << std::endl;
	return db_tgt;
}

int main(int argc, char** argv)
{
	//Experiment conditions
	const int tactile_period_ms = 600;
	const int num_series = 7;
	const int num_stimulus = 3;
	int freq = 200;
	const float db_ref = -6;
	const float db_max = -3.2;
	const float db_min = db_ref;

	std::string prefix_log;
	std::cout << "Enter the participant's name:";
	std::cin >> prefix_log;
	std::cout << std::endl << "Name: " << prefix_log << std::endl;
	std::cout << "Now initializing ..." << std::endl;

	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen())
		return ENXIO;
	haptic_icon::SetGeometry(pAupa);

	std::string target_image_name("blue_target_no_cover.png");
	auto pTracker = haptic_icon::CreateTracker(target_image_name);

	auto pObject = dynaman::FloatingObject::Create(
		Eigen::Vector3f(0, 0, 0),
		Eigen::Vector3f::Constant(-400),
		Eigen::Vector3f::Constant(400),
		-0.036e-3f,
		50.f
	);
	Vibrator vibrator(prefix_log, pAupa, pTracker);
	vibrator.SetAmplitude(0.0f);
	vibrator.SetFrequency(200);
	vibrator.Start(pObject);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	vibrator.Stop();

	std::cout << "Press enter to start." << std::endl;
	getchar();	
	getchar();
	std::ofstream ofs_all(prefix_log + "_all.txt", std::ios_base::app);
	std::ofstream ofs_rev_points(prefix_log + "_rev.txt", std::ios_base::app);
	float db_rev = db_max;
	std::cout << "Current frqeuency is: " << freq << std::endl;
	for (int i_series = 0; i_series < num_series; i_series++) {
		float db_step = (i_series == 0 ? 0.4 : 0.1);
		float db_init_series;
		if (i_series == 0)
		{
			db_init_series = db_max;
		}
		else
		{
			db_init_series = db_rev + (i_series % 2 == 0 ? -db_step : db_step);
		}
		std::cout << "i_series : " << i_series << std::endl;
		std::cout << "db_rev : " << db_rev << std::endl;
		std::cout << "db_init_series : " << db_init_series << std::endl;
		db_rev = run_series(
			vibrator,
			db_ref,
			db_init_series,
			db_min,
			db_max,
			db_step,
			freq,
			tactile_period_ms,
			num_stimulus,
			pObject,
			i_series % 2 == 0 ? Series::DESCEND : Series::ASCEND,
			ofs_all
		);
		if (i_series != 0) {
			ofs_rev_points << db_rev << ",";
		}
	}	
	ofs_all.close();
	ofs_rev_points.close();
	pAupa->Close();
	return 0;
}