#include <iostream>
#include "GainPlan.hpp"

using namespace dynaman;

int main(int argc, char** argv) {
	auto cs = combination(3, 3);
	for (auto&& c : cs) {
		for (auto&& e : c) {
			std::cout << e << ",";
		}
		std::cout << std::endl;
	}
	return 0;
}