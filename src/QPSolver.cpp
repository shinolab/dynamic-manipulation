#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/QP_solution.h>
#include <Eigen/Dense>
#include <iostream>

// choose exact integral type
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpz ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif

typedef CGAL::Linear_program_from_iterators
<const float**, const float*, CGAL::Comparison_result*, const bool*, const float*, const bool*, const float*, const float*> Program;

typedef CGAL::Quadratic_program_solution<ET> Solution;
/*
Solve Linear Programming minimize: y = c'x subject to Ax </=/> b, .
sign of the coefficients of equality conditions specifies inequality/equlaity conditions of the row number.
*/
void EigenLinearProgrammingSolver(
	Eigen::VectorXf &result,
	Eigen::MatrixXf const &A,
	Eigen::VectorXf const &b,
	Eigen::VectorXf const &c,
	Eigen::VectorXi const &equalityConditions,
	Eigen::VectorXf const &lowerbound,
	Eigen::VectorXf const &upperbound
)
{
	const float** _A = new const float*[A.cols()];
	for (int i = 0; i < A.cols(); i++)
	{
		_A[i] = A.data() + i * (A.rows());
	}
	CGAL::Comparison_result* r = new CGAL::Comparison_result[A.rows()];
	bool* flb = new bool[A.cols()];
	bool* fub = new bool[A.cols()];

	for (int i = 0; i < A.cols(); i++)
	{
		flb[i] = true;
		fub[i] = true;
	}

	for (int iCond = 0; iCond < A.rows(); iCond++)
	{
		if (equalityConditions[iCond] < 0)
			r[iCond] = CGAL::SMALLER;
		else if (equalityConditions[iCond] > 0)
			r[iCond] = CGAL::LARGER;
		else
			r[iCond] = CGAL::EQUAL;
	}
	Program lp(A.cols(), A.rows(), _A, b.data(), r, flb, lowerbound.data(), fub, upperbound.data(), c.data(), 0.f);
	Solution s = CGAL::solve_linear_program(lp, ET());
	result.resize(A.cols());
	for (auto itr = s.variable_values_begin(); itr != s.variable_values_end(); itr++)
	{
		int index = std::distance(s.variable_values_begin(), itr);
		result[index] = (*itr).numerator().to_double() / (*itr).denominator().to_double();
	}

	delete _A;
	delete fub;
	delete flb;
	delete r;
}

//define QP

