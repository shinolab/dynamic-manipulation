#include "odcs.hpp"
#include <iostream>
#include <fstream>
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>

void amplitudeConversionTest()
{
	std::ofstream ofs("20190130_duty_conversion3.csv");
	ofs << "u0, u1, u2, u3, u4, amp0, amp1, amp2, amp3, amp4" << std::endl;
	for (int i = 0; i < 100; i++)
	{
		Eigen::VectorXf duties(5); duties.setRandom();
		Eigen::VectorXf dutiesAbs = duties.cwiseAbs();
		duties /= (0.5 * dutiesAbs.maxCoeff());
		//duties += 0.5*duties.cwiseAbs(); 
		Eigen::VectorXi amplitudes = (510 / M_PI * (duties.array().max(0.f).min(1.f).sqrt().asin())).cast<int>().matrix();
		for (int j = 0; j < 5; j++)
		{
			ofs << duties[j] << ", ";
		}
		for (int j = 0; j < 5; j++)
		{
			ofs << amplitudes[j] << ", ";
		}
		ofs << std::endl;
	}
	ofs.close();
}

void setRandomRange(Eigen::Vector3f &result, Eigen::Vector3f const &corner1, Eigen::Vector3f const &corner2)
{
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());
	std::uniform_real_distribution<> dist(0.f, 1.f);
	result = corner1 + (corner2 - corner1) * dist(engine);
}

int main()
{
	odcs odcs;
	odcs.Initialize();
	//auto objPtr = FloatingObject::Create(Eigen::Vector3f(0, 0, 1500));
	Eigen::Vector3f pos(50, 50, 1223);
	std::cout << "centersAUTD:\n " << odcs.ocs.centersAUTD << std::endl;
	//objPtr->updateStates(timeGetTime(), pos);
	//odcs.ocs.arfModelPtr->arf(objPtr->getPosition(), odcs.ocs.eulerAnglesAUTD);
	Eigen::MatrixXf constraint(3, 2); constraint << Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY();
	Eigen::VectorXf duty_limit(5); duty_limit.setConstant(0.8f);
	Eigen::VectorXf duties = odcs.ocs.FindDutyMaximizeForce(Eigen::Vector3f::UnitZ(), constraint, pos, duty_limit);
	std::cout << duties.transpose() << std::endl;
	return 0;
}