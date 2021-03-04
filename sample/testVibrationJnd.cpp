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

Answer ask(bool ref_first) {
	Answer ans;
	while (1) {
		std::cout << "Ask which one is STRONGER ? [first/second/same]" << std::endl;
		std::string key;
		std::cin >> key;
		if (key == "first") {
			ans = (ref_first ? Answer::EVAL_WEAK : Answer::EVAL_STRONG);
			break;
		}
		else if (key == "second") {
			ans = (ref_first ? Answer::EVAL_STRONG : Answer::EVAL_WEAK);
			break;
		}
		else if (key == "same") {
			ans = Answer::SAME;
			break;
		}
	}
	return ans;
}

void present_stimulation(
	Vibrator& vibrator,
	float first_amp,
	float second_amp,
	int freq,
	int duration_ms,
	FloatingObjectPtr pObject)
{
	std::cout << "current amp is: " << first_amp << " (" << Vibrator::amp_to_decibel(first_amp) << "(dB)). press q to abort, or press n to ascend." << std::endl;
	vibrator.SetAmplitude(first_amp);
	vibrator.SetFrequency(freq);
	Beep(440, 500);
	std::this_thread::sleep_for(std::chrono::milliseconds(400));
	vibrator.Start(pObject);
	std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
	vibrator.Stop();
	std::this_thread::sleep_for(std::chrono::milliseconds(400));
	vibrator.SetAmplitude(second_amp);
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

void run_series(
	Vibrator& vibrator,
	float db_ref,
	float db_min,
	float db_max,
	float db_step,
	int freq,
	int duration_ms,
	bool is_ref_first,
	FloatingObjectPtr pObject,
	Series series,
	std::ofstream& ofs
) {
	ofs << series_to_string(series) <<",";
	float db_init, db_end;
	if (series == Series::ASCEND) {
		db_init = db_min;
		db_end = db_max;
	}
	else if (series == Series::DESCEND) {
		db_init = db_max;
		db_end = db_min;
	}
	for (float db_eval = db_init; (db_eval - db_init) * (db_eval - db_end) <= 0; change_db(db_eval, db_step, series)){
		float amp_ref = Vibrator::decibel_to_amp(db_ref);
		float amp_eval = Vibrator::decibel_to_amp(db_eval);
		if (is_ref_first) {
			present_stimulation(vibrator, amp_ref, amp_eval, freq, duration_ms, pObject);
		}
		else {
			present_stimulation(vibrator, amp_eval, amp_ref, freq, duration_ms, pObject);
		}
		ofs << db_ref << ",(" << amp_ref << ")," << db_eval << ",(" << amp_eval << ")";
		bool detected = false;
		auto answer = ask(is_ref_first);
		ofs << "," << answer_to_string(answer);
		if (is_final(answer, series)) {
			Beep(800, 400);
			ofs << std::endl;
			break;
		}
	}
	while (1) {
		std::cout << "Take a rest. press 'go' to proceed." << std::endl;
		std::string key;
		std::cin >> key;
		if (key == "go") {
			break;
		}
	}
}

int main(int argc, char** argv)
{
	//Experiment conditions
	const float db_step = 1;
	const int num_trial = 3;
	const int tactile_period_ms = 1000;
	std::vector<int> freq_list{ 200 };
	std::vector<float> db_ref_list{ -13 };
	std::vector<float> db_offset_max_list{ 6, 7, 8 };
	std::vector<float> db_offset_min_list{ 5, 6, 7 };

	bool ref_first = true;
	std::string ref_order;
	std::cout << "Press 1 for reference first, Press 2 for reference second." << std::endl;
	std::cin >> ref_order;
	if (ref_order == "1") {
		std::cout << "A reference-FIRST sequence is selected." << std::endl;
		ref_first = true;
	}
	else if (ref_order == "2") {
		std::cout << "A reference-SECOND sequence is selected." << std::endl;
		ref_first = false;
	}
	else {
		std::cout << "Invalid number is detected. Aborting..." << std::endl;
		return -1;
	}
	std::string prefix_log;
	std::cout << "Enter the participant's name:";
	std::cin >> prefix_log;
	std::cout << std::endl << "Name: " << prefix_log << std::endl;
	prefix_log += (ref_first ? "_rf_" : "_rs_");
	std::cout << "Now initializing ..." << std::endl;

	if (num_trial != db_offset_max_list.size() || num_trial != db_offset_min_list.size()) {
		std::cerr << "error found in configuration parameter." << std::endl;
		return -1;
	}
	if (freq_list.size() != db_ref_list.size()) {
		std::cerr << "error found in freq parameter." << std::endl;
		return -1;
	}
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());

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
	vibrator.Start(pObject);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	vibrator.Stop();
	std::ofstream ofs(prefix_log + "_jnd.txt", std::ios_base::app);
	for (auto itr_freq = freq_list.begin(); itr_freq != freq_list.end(); itr_freq++) 
	{
		int idx_freq = std::distance(freq_list.begin(), itr_freq);
		auto freq = *itr_freq;

		std::shuffle(db_offset_max_list.begin(), db_offset_max_list.end(), engine);
		std::shuffle(db_offset_min_list.begin(), db_offset_min_list.end(), engine);
		std::cout << "Current frqeuency is: " << freq << std::endl;
		for (int i_trial = 0; i_trial < num_trial; i_trial++) {
			//ascending series
			std::cout << "Ascending Series has started." << std::endl;
			auto db_ref = db_ref_list[idx_freq];
			float db_min = db_ref - db_offset_min_list[i_trial];
			float db_max = db_ref + db_offset_max_list[i_trial];
			run_series(
				vibrator,
				db_ref,
				db_min,
				db_max,
				db_step,
				freq,
				tactile_period_ms,
				ref_first,
				pObject,
				Series::ASCEND,
				ofs
			);

			//descending series
			std::cout << "Descending Series has started." << std::endl;
			run_series(
				vibrator,
				db_ref,
				db_min,
				db_max,
				db_step,
				freq,
				tactile_period_ms,
				ref_first,
				pObject,
				Series::ASCEND,
				ofs
			);
		}
		
	}
	return 0;
}