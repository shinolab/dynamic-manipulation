#include "manipulator.hpp"
#include "autd3.hpp"
#include <iostream>
#include <random>
#include "haptic_icon.hpp"

int main(int argc, char** argv) {

	/*user-defined configurations*/
	Eigen::Vector3f pos(0, 0, 0);
	std::string target_image_name("blue_target_r50mm.png");
	/*end of user-defined configurations*/

	auto pTracker = haptic_icon::CreateTracker(target_image_name);
	pTracker->open();

	auto pAupa = std::make_shared<autd::Controller>();
	pAupa->Open(autd::LinkType::ETHERCAT);
	if (!pAupa->isOpen())
		return ENXIO;
	haptic_icon::SetGeometry(pAupa);

	auto pObject = dynaman::FloatingObject::Create(
		pos,
		Eigen::Vector3f::Constant(-600),
		Eigen::Vector3f::Constant(600),
		0,
		50.f
	);

	Eigen::Vector3f gainP(1, 1, 1);
	Eigen::Vector3f gainD(1, 1, 1);
	Eigen::Vector3f gainI(0, 0, 0);

	int period_mux_us = 1000 * 1000 * 0.01;

	dynaman::VarMultiplexManipulator manipulator(
		gainP,
		gainD,
		gainI,
		period_mux_us,
		33,
		10,
		0,
		std::make_shared<arfModelFocusSphereExp50mm>()
	);


	manipulator.m_pAupa = pAupa;
	manipulator.m_pTracker = pTracker;
	manipulator.m_pObject = pObject;
	
	int num_trial = 5;
	int update_interval_ms = 10000;

	//create random duty
	
	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());

	std::uniform_int_distribution dist(0, pAupa->geometry()->numDevices() - 1);

	pAupa->AppendModulationSync(autd::SineModulation::Create(150));


	for (int i_trial = 0; i_trial < num_trial; i_trial++) {
		std::cout << "Trial #" << i_trial << std::endl;
		Eigen::Vector3f focus(0, 0, 0);
		Eigen::VectorXf duty(pAupa->geometry()->numDevices());
		duty.setZero();
		//for (int j = 0; j < 4; j++) {
		//	duty[dist(engine)] = 1;
		//}
		duty[0] = 1;
		duty[1] = 1;
		duty[2] = 1;
		duty.normalize();
		std::cout << "duty: " << duty.transpose() << std::endl;
		auto sequence = manipulator.CreateDriveSequence(
			duty,
			focus
		);
		std::cout << "Append Gain" << std::endl;

		for (int i = 0; i < sequence.size(); i++) {
			std::cout << "TimeSlices: " << sequence[i].second << ", ";
			manipulator.mux.AddOrder(
				[sequence, i, &pAupa]() {
					std::cout << "Append Gain #" << i << std::endl;
					pAupa->AppendGainSync(sequence[i].first);
				},
				sequence[i].second
			);
		}
		std::cout << std::endl;
		manipulator.mux.Start();
		std::this_thread::sleep_for(std::chrono::milliseconds(update_interval_ms));
		manipulator.mux.Stop();
		manipulator.mux.ClearOrders();


	}
	pAupa->Close();
	return 0;
}