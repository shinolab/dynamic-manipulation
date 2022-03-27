#include "MathUtil.hpp"

namespace dynaman {

	std::vector<std::vector<size_t>> combination(size_t max, size_t num) {
		if (num == 1) {
			std::vector<std::vector<size_t>> v;
			for (size_t i = 0; i <= max; i++) {
				v.push_back({ i });
			}
			return v;
		}
		if (num == max + 1) {
			std::vector<size_t> v;
			for (int i = 0; i <= max; i++) {
				v.push_back(i);
			}
			return { v };
		}
		auto others = combination(max - 1, num);
		auto inc = combination(max - 1, num - 1);
		for (auto itr = inc.begin(); itr != inc.end(); itr++) {
			itr->push_back(max);
		}
		inc.insert(inc.end(), others.begin(), others.end());
		return inc;
	}

}