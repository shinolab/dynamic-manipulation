#include <boost/math/tools/roots.hpp>

#include <boost/math/special_functions/next.hpp>
#include <tuple>
#include <boost/math/special_functions/cbrt.hpp>

#include <iostream>

template<class T>
struct quarticPoly_deriv
{
	quarticPoly_deriv(T const &a0, T const &a1, T const &a2, T const &a3, T const &a4) : _a0(a0), _a1(a1), _a2(a2), _a3(a3), _a4(a4) {}

	std::pair<T, T> operator()(T const &x)
	{
		T f = (((_a4*x + _a3) * x + _a2)*x + _a1)*x + _a0;
		T dfdx = ((4 * _a4 * x + 3 * _a3) * x + 2 * _a2)*x + _a1;
		return std::make_pair(f, dfdx);
	}
private:
	T _a0, _a1, _a2, _a3, _a4;
};

template<class T>
T findRootQuarticPoly(T const &a0
	, T const &a1
	, T const &a2
	, T const &a3
	, T const &a4
	, T const &guess
	, T const &min
	, T const &max
	, int digits = 0.6 * std::numeric_limits<T>::digits)
{
	using namespace boost::math::tools;
	boost::uintmax_t max_iter = 20;
	return newton_raphson_iterate(quarticPoly_deriv<T>(a0, a1, a2, a3, a4), guess, min, max, digits, max_iter);
}